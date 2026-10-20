#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Native Python 2 / existing OpenCV checks for all original QR assets."""
from __future__ import print_function
import cv2,hashlib,imp,itertools,json,os,sys,time
import rospy
from pyzbar.pyzbar import decode
ROOT=os.path.dirname(os.path.abspath(__file__))
SCRIPTS='/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts'
LEGAL=['A','B','C','AB','AC','BC','ABC']

def expected(values):
    candidates=[(len(v),-i,i,v) for i,v in enumerate(values) if v in LEGAL]
    if not candidates:return None
    _,_,i,v=max(candidates)
    error=next((j+1 for j,t in enumerate(values) if t and t not in LEGAL),0)
    return [int('C' in v),int('A' in v),int('B' in v),len(v),i,error]

class Recorder(object):
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(msg.data if hasattr(msg,'data') else msg)

def main():
    manifest=json.load(open(ROOT+'/manifest.json'))
    sys.path.insert(0,SCRIPTS)
    m=imp.load_source('board1_full_qr_verification',SCRIPTS+'/F1_detect_code_v5.py')
    for key in ['loginfo','logwarn','loginfo_throttle','logwarn_throttle','sleep']:
        setattr(rospy,key,lambda *a,**k:None)
    m.pub_flag=Recorder();m.pub_board1_all_text=Recorder()
    report={'assets':[],'isolated':[],'selection_combinations':0,'success':False}
    try:
        for name,meta in sorted(manifest['assets'].items()):
            p=ROOT+'/assets/'+name
            assert hashlib.sha256(open(p,'rb').read()).hexdigest()==meta['sha256']
            codes=decode(cv2.imread(p));texts=[c.data.decode('utf-8') for c in codes]
            assert texts==([meta['payload']] if meta['payload'] else []),(name,texts)
            report['assets'].append({'file':name,'decoded':texts})
        for spec in manifest['isolated']:
            frame=cv2.resize(cv2.imread(ROOT+'/isolated/'+spec['case']+'.png'),(640,360),interpolation=cv2.INTER_AREA)
            got=m.decode_board1_by_windows(frame)
            row={'case':spec['case'],'all_text':got,'expected':spec['all_text']}
            report['isolated'].append(row)
            assert got==spec['all_text'],row
            select=m.select_board1_from_all_text(got)
            msg=select['selected_msg'] if select else None
            assert msg==expected(spec['all_text']),(row,msg)
        options=['']+LEGAL+['ACD','D','https://www.hlcode.cn']
        for values in itertools.product(options,repeat=4):
            result=m.select_board1_from_all_text(values)
            assert (result['selected_msg'] if result else None)==expected(values),values
            report['selection_combinations']+=1
        report['success']=True
    except Exception as error:report['error']=repr(error)
    finally:
        json.dump(report,open(ROOT+'/native_results.json','w'),indent=2)
    print(json.dumps(dict(success=report['success'],error=report.get('error'),assets=len(report['assets']),isolated=len(report['isolated']),combinations=report['selection_combinations'])))
    return 0 if report['success'] else 1

if __name__=='__main__':sys.exit(main())
