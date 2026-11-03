#!/usr/bin/env python3
"""Private Python3 ONNX/NCNN worker. JSON IPC only; no ROS, TCP or audio."""
import argparse
import ast
import base64
from collections import deque
import datetime
import hashlib
import json
import os
import sys
import time
import cv2
import numpy as np
from board2_yolo_locator import find_black_frame, align_board_to_training_coordinates


def _check_tensor(tensor):
    if tensor.shape != (1,3,224,224) or tensor.dtype != np.float32 or not np.isfinite(tensor).all():
        raise ValueError('Expected finite FP32 [1,3,224,224] classifier input')


def _check_probabilities(values, size):
    values=np.asarray(values,dtype=np.float32).reshape(-1).copy()
    if (values.shape != (size,) or not np.isfinite(values).all()
            or values.min() < -1e-6 or values.max() > 1.000001
            or abs(float(values.sum())-1.0) > .001):
        raise ValueError('Invalid classifier probability output')
    return values


class OnnxClassifier:
    def __init__(self,model_dir,name,size,threads):
        import onnxruntime as ort
        options=ort.SessionOptions()
        options.intra_op_num_threads=threads
        options.inter_op_num_threads=1
        self.session=ort.InferenceSession(os.path.join(model_dir,name+'_best.onnx'),
                                        sess_options=options,providers=['CPUExecutionProvider'])
        self.size=size
        if self.session.get_inputs()[0].shape!=[1,3,224,224] or self.session.get_outputs()[0].shape!=[1,size]:
            raise ValueError('Unexpected model shape: '+name)
        names=ast.literal_eval(self.session.get_modelmeta().custom_metadata_map['names'])
        self.classes=[str(names[i]) for i in range(size)]
        self.input_name=self.session.get_inputs()[0].name

    def predict(self,tensor):
        _check_tensor(tensor)
        return _check_probabilities(self.session.run(None,{self.input_name:tensor})[0],self.size)


class NcnnClassifier:
    def __init__(self,model_dir,name,size,threads):
        import ncnn
        self.ncnn=ncnn
        self.size=size
        with open(os.path.join(model_dir,'ncnn_manifest.json')) as f: manifest=json.load(f)
        entry=manifest['models'][name]
        if manifest['precision']!='fp32' or manifest['input']!=[1,3,224,224] or entry['output_size']!=size:
            raise ValueError('Unexpected NCNN model contract: '+name)
        self.classes=entry['classes']
        for filename,digest in entry['sha256'].items():
            if os.path.basename(filename)!=filename: raise ValueError('Invalid manifest filename')
            with open(os.path.join(model_dir,filename),'rb') as f:
                if hashlib.sha256(f.read()).hexdigest()!=digest:
                    raise ValueError('NCNN model fingerprint mismatch: '+filename)
        self.net=ncnn.Net()
        # Measured on Cortex-A72: avoid transformed convolution weight buffers.
        self.net.opt.use_winograd_convolution=False
        self.net.opt.use_sgemm_convolution=False
        self.net.opt.num_threads=threads
        self.net.opt.use_vulkan_compute=False
        self.net.opt.use_fp16_packed=False
        self.net.opt.use_fp16_storage=False
        self.net.opt.use_fp16_arithmetic=False
        self.net.opt.use_bf16_storage=False
        if self.net.load_param(os.path.join(model_dir,name+'_best.ncnn.param'))!=0:
            raise ValueError('NCNN param load failed: '+name)
        if self.net.load_model(os.path.join(model_dir,name+'_best.ncnn.bin'))!=0:
            raise ValueError('NCNN weights load failed: '+name)
        if list(self.net.input_names())!=[entry['input_name']] or list(self.net.output_names())!=[entry['output_name']]:
            raise ValueError('Unexpected NCNN blob names: '+name)
        self.input_name=entry['input_name']
        self.output_name=entry['output_name']

    def predict(self,tensor):
        _check_tensor(tensor)
        # Keep the contiguous CHW array alive through extraction; NCNN Mat shares its memory.
        chw=np.ascontiguousarray(tensor[0])
        with self.net.create_extractor() as extractor:
            if extractor.input(self.input_name,self.ncnn.Mat(chw))!=0:
                raise ValueError('NCNN input failed')
            code,output=extractor.extract(self.output_name)
            if code!=0: raise ValueError('NCNN output extraction failed')
            return _check_probabilities(np.asarray(output),self.size)


def load_classifier(model_dir,name,size,backend,threads):
    if threads not in (1,2): raise ValueError('Board2 threads must be 1 or 2')
    if backend=='onnx': return OnnxClassifier(model_dir,name,size,threads)
    if backend=='ncnn': return NcnnClassifier(model_dir,name,size,threads)
    raise ValueError('Board2 backend must be onnx or ncnn')


def business_label(status, number, minimum, margin):
    status=np.asarray(status)
    number=np.asarray(number)
    if not np.isfinite(status).all() or not np.isfinite(number).all(): return None,0.0
    si=int(status.argmax())
    score=float(status[si])
    if score<minimum or float(abs(status[0]-status[1]))<margin: return None,score
    if si==1: return 'free',score
    ni=int(number.argmax())
    ordered=np.sort(number)
    score=min(score,float(number[ni]))
    if score<minimum or float(ordered[-1]-ordered[-2])<margin: return None,score
    return 'busy_'+['10','5','6','7','8','9'][ni],score


def prepared_tensor(image,roi):
    x,y,w,h=roi
    crop=image[y:y+h,x:x+w]
    if crop.size==0 or crop.shape[:2]!=(h,w): raise ValueError('ROI outside aligned board')
    side=max(h,w)
    square=np.full((side,side,3),255,dtype=np.uint8)
    yy,xx=(side-h)//2,(side-w)//2
    square[yy:yy+h,xx:xx+w]=crop
    resized=cv2.resize(square,(224,224),interpolation=cv2.INTER_LINEAR)
    rgb=cv2.cvtColor(resized,cv2.COLOR_BGR2RGB)
    return np.ascontiguousarray(rgb.transpose(2,0,1)[None],dtype=np.float32)/255.0


def normalize_number_row(board):
    """Align the lower text row to dataset geometry, without consulting digit labels.

    Target center x=236, top=160 and height=42 are median geometry of the
    original 360 busy-board images. Preserve aspect ratio, then remove the
    camera's colored edges; the original model, crop and thresholds stay fixed.
    """
    if board.shape[:2]!=(320,480): raise ValueError('Unexpected aligned dimensions')
    mask=(cv2.cvtColor(board,cv2.COLOR_BGR2GRAY)<110).astype(np.uint8)
    mask[:145]=0; mask[240:]=0; mask[:,:75]=0; mask[:,405:]=0
    ys=np.flatnonzero(mask.sum(axis=1)>=8)
    groups=np.split(ys,np.flatnonzero(np.diff(ys)>4)+1)
    groups=[g for g in groups if len(g)>15]
    if not groups: raise ValueError('No complete lower text row')
    group=max(groups,key=len)
    y1,y2=int(group[0]),int(group[-1])+1
    xs=np.flatnonzero(mask[y1:y2].sum(axis=0)>=3)
    x,y,w,h=int(xs[0]),y1,int(xs[-1])+1-int(xs[0]),y2-y1
    if not (150<=w<=330 and 20<=h<=70): raise ValueError('Invalid lower text geometry')
    scale=42.0/h
    matrix=np.array([[scale,0,236-scale*(x+w/2.0)],
                     [0,scale,160-scale*y]],dtype=np.float32)
    normalized=cv2.warpAffine(board,matrix,(480,320),borderValue=(255,255,255))
    gray=cv2.cvtColor(normalized,cv2.COLOR_BGR2GRAY)
    return cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR),[x,y,w,h]


class Recognizer:
    def __init__(self,model_dir,evidence_dir,backend='onnx',threads=2):
        self.model_dir=model_dir
        self.evidence_dir=evidence_dir
        os.makedirs(evidence_dir,exist_ok=True)
        with open(os.path.join(model_dir,'model_contract.json')) as f: self.contract=json.load(f)
        if self.contract['classes']!={'status':['busy','idle'],'number':['10','5','6','7','8','9']}:
            raise ValueError('Unexpected model class order')
        self.backend=backend
        self.threads=threads
        if backend=='ncnn':
            # This worker handles Board2 only; the ROS/Board1 process is unaffected.
            cv2.setNumThreads(1)
        load_started=time.perf_counter()
        self.sessions={}
        for name,size in [('status',2),('number',6)]:
            session=load_classifier(model_dir,name,size,backend,threads)
            if session.classes!=self.contract['classes'][name]:
                raise ValueError('Model class metadata differs from contract: '+name)
            self.sessions[name]=session
        self.load_ms=(time.perf_counter()-load_started)*1000.0
        self.minimum=float(self.contract.get('minimum_confidence',0.90))
        self.margin=float(self.contract.get('minimum_margin',0.15))
        self.window=int(self.contract.get('smoothing_frames',1))
        self.number_normalization=self.contract.get('number_normalization')
        if self.number_normalization not in (None,'text_row_gray_v1'):
            raise ValueError('Unknown number normalization')
        self.history={name:deque(maxlen=self.window) for name in self.sessions}
        self.visit=None
        self.snapshot_visit=None
        self.log=open(os.path.join(evidence_dir,'yolo_frames.jsonl'),'a',buffering=1)

    def reset(self):
        for values in self.history.values(): values.clear()

    def warmup(self):
        """Prepare both model paths without adding observations to a visit."""
        started=time.perf_counter()
        tensor=np.zeros((1,3,224,224),dtype=np.float32)
        for name in ('status','number'):
            self.sessions[name].predict(tensor)
        return {'warmup_runs':2,'warmup_ms':(time.perf_counter()-started)*1000.0}

    def close(self):
        self.log.close()

    def infer(self,frame,visit):
        started=time.perf_counter()
        if visit!=self.visit:
            self.visit=visit
            self.snapshot_visit=None
            self.reset()
        row={'timestamp':datetime.datetime.now().isoformat(),'epoch':time.time(),
             'visit':visit,'label':None,'score':0.0,'history_frames':0}
        inference_ms=0.0
        preprocess_ms=0.0
        preprocessing_started=time.perf_counter()
        candidate,count,_=find_black_frame(frame,110)
        aligned=None if candidate is None else align_board_to_training_coordinates(frame,candidate.points)
        preprocess_ms+=(time.perf_counter()-preprocessing_started)*1000.0
        row['candidate_count']=count
        row['board_found']=aligned is not None
        if aligned is None:
            self.reset()
            row['reason']='no_board'
        else:
            board=aligned.aligned_board
            row['points']=candidate.points.tolist()
            for name,session in self.sessions.items():
                preprocessing_started=time.perf_counter()
                input_board=board
                if name=='number' and self.number_normalization and int(np.argmax(row['status_raw_probabilities']))==0:
                    try:
                        input_board,geometry=normalize_number_row(board)
                    except ValueError as exc:
                        self.reset()
                        row['reason']='invalid_number_geometry'
                        row['detail']=str(exc)
                        row['inference_ms']=(time.perf_counter()-started)*1000.0
                        self.log.write(json.dumps(row,ensure_ascii=True)+'\n')
                        return row
                    row['number_text_geometry']=geometry
                    row['number_normalization']=self.number_normalization
                tensor=prepared_tensor(input_board,self.contract['roi'][name])
                preprocess_ms+=(time.perf_counter()-preprocessing_started)*1000.0
                inference_started=time.perf_counter()
                probabilities=session.predict(tensor)
                inference_ms+=(time.perf_counter()-inference_started)*1000.0
                if not np.isfinite(probabilities).all() or abs(float(probabilities.sum())-1.0)>.001:
                    raise ValueError('Invalid classifier probabilities')
                self.history[name].append(probabilities)
                smoothed=np.mean(np.stack(self.history[name]),axis=0)
                row[name+'_raw_probabilities']=probabilities.tolist()
                row[name+'_probabilities']=smoothed.tolist()
            row['history_frames']=len(self.history['status'])
            label,score=business_label(row['status_probabilities'],row['number_probabilities'],self.minimum,self.margin)
            row['score']=score
            row['reason']='warmup' if row['history_frames']<self.window else 'low_confidence'
            if row['history_frames']>=self.window and label is not None:
                row['label']=label
                row['reason']='accepted'
                if self.snapshot_visit!=visit:
                    self.save_snapshot(frame,board,row)
                    self.snapshot_visit=visit
        row['inference_ms']=(time.perf_counter()-started)*1000.0
        row['timings_ms']={'preprocess':preprocess_ms,'model_inference':inference_ms,
                           'postprocess_and_evidence':max(0.0,row['inference_ms']-preprocess_ms-inference_ms)}
        self.log.write(json.dumps(row,ensure_ascii=True)+'\n')
        return row

    def save_snapshot(self,frame,board,row):
        name=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_visit_'+str(row['visit'])
        prefix=os.path.join(self.evidence_dir,name)
        cv2.imwrite(prefix+'_raw.png',frame)
        annotated=frame.copy()
        cv2.polylines(annotated,[np.round(row['points']).astype(np.int32)],True,(0,255,0),2)
        cv2.putText(annotated,'%s %.5f'%(row['label'],row['score']),(12,430),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,0),2)
        cv2.putText(annotated,row['timestamp'],(12,460),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
        cv2.imwrite(prefix+'_annotated.png',annotated)
        cv2.imwrite(prefix+'_aligned.png',board)
        if row.get('number_text_geometry'):
            normalized,_=normalize_number_row(board)
            x,y,w,h=self.contract['roi']['number']
            cv2.imwrite(prefix+'_number_input.png',normalized[y:y+h,x:x+w])
        with open(prefix+'_result.json','w') as f: json.dump(row,f,indent=2)
        row['snapshot_prefix']=prefix


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--model-dir',required=True)
    parser.add_argument('--evidence-dir',required=True)
    parser.add_argument('--backend',choices=['onnx','ncnn'],default='onnx')
    parser.add_argument('--threads',type=int,choices=[1,2],default=2)
    args=parser.parse_args()
    recognizer=Recognizer(args.model_dir,args.evidence_dir,args.backend,args.threads)
    warmup=recognizer.warmup()
    print(json.dumps({'ready':True,'runtime':sys.version,'backend':args.backend,
                      'threads':args.threads,'load_ms':recognizer.load_ms,
                      'warmup_runs':warmup['warmup_runs'],'warmup_ms':warmup['warmup_ms']}),flush=True)
    for line in sys.stdin:
        try:
            request=json.loads(line)
            if request.get('op')=='shutdown': break
            data=np.frombuffer(base64.b64decode(request['image']),dtype=np.uint8)
            frame=cv2.imdecode(data,cv2.IMREAD_COLOR)
            if frame is None: raise ValueError('Invalid camera JPEG')
            row=recognizer.infer(frame,request['visit'])
        except Exception as exc:
            recognizer.reset()
            row={'label':None,'score':0.0,'reason':'worker_error','error':repr(exc)}
        print(json.dumps(row,ensure_ascii=True),flush=True)
    recognizer.close()


if __name__=='__main__': main()
