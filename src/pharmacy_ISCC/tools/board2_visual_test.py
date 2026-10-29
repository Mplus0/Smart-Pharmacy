#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Stationary isolated Board2 rearm and speech observer. No navigation objects."""
import json
import os
import threading
import time


class OncePerVisit(object):
    def __init__(self):
        self.lock=threading.Lock()
        self.visit=0
        self.armed=False
        self.used=False
        self.speaking=False

    def rearm(self):
        with self.lock:
            if self.speaking:return False
            self.visit+=1
            self.armed=False
            self.used=False
            return self.visit

    def enable(self,visit,publish=None):
        with self.lock:
            if visit!=self.visit or self.armed or self.used or self.speaking:return False
            self.armed=True
            if publish is not None:publish()
            return True

    def take(self,data):
        if len(data)!=2:return None
        state,wait=data
        if not ((state==0 and wait==0) or (state==1 and 5<=wait<=10)):return None
        with self.lock:
            if not self.armed or self.used or self.speaking:return None
            self.used=True
            self.speaking=True
            return self.visit,'WAIT-%d.wav'%wait

    def finish(self):
        with self.lock:self.speaking=False


def main():
    import rospy
    from std_msgs.msg import Int32,Int32MultiArray
    from std_srvs.srv import Trigger,TriggerResponse
    from pharmacy_iscc.audio import play_audio
    from pharmacy_iscc.config import records_path
    from pharmacy_iscc.F1_yaofang_v5 import MoveBaseSquare
    rospy.init_node('board2_visual_audio')
    if int(rospy.get_param('~state',13))!=13:raise ValueError('Board2 speech test requires state 13')
    evidence=records_path()
    if not os.path.isdir(evidence):os.makedirs(evidence)
    log=open(os.path.join(evidence,'stationary_events.jsonl'),'a',0)
    log_lock=threading.Lock()
    gate=OncePerVisit()
    state_pub=rospy.Publisher('/pharmacy_vision_debug/nav_state',Int32,queue_size=10,latch=True)
    last={'result':None,'speech_calls':0,'speech_finished':0}
    def record(event,**values):
        row=dict(event=event,time=time.time(),visit=gate.visit,**values)
        with log_lock:log.write((json.dumps(row,ensure_ascii=True)+'\n').encode('ascii'))
        rospy.loginfo('Board2 stationary %s',json.dumps(row))
    class Receiver(object):pass
    class Records(object):
        def record(self,*args):record('business_record',values=args)
    receiver=Receiver()
    receiver.count=13
    receiver.board2_received=False
    receiver.recognition_records=Records()
    # Reuse the exact callback function without constructing navigation objects.
    business_callback=getattr(MoveBaseSquare.detect_board2_result,'im_func',MoveBaseSquare.detect_board2_result)
    def enable(event,visit):
        if not gate.enable(visit,lambda:state_pub.publish(Int32(13))):return
        record('armed')
    def rearm(request):
        visit=gate.rearm()
        if not visit:return TriggerResponse(False,'Speech is still playing')
        last['result']=None
        receiver.board2_received=False
        state_pub.publish(Int32(0))
        rospy.Timer(rospy.Duration(.35),lambda event:enable(event,visit),oneshot=True)
        record('rearming')
        return TriggerResponse(True,'Arming visit %d'%gate.visit)
    def speak(visit,filename,data):
        last['speech_calls']+=1
        record('speech_start',filename=filename,data=data)
        try:
            play_audio(filename)
            last['speech_finished']+=1
            record('play_call_finished',filename=filename,data=data,audibility='requires microphone or human confirmation')
        except Exception as exc:record('play_error',error=repr(exc))
        finally:gate.finish()
    def result(message):
        data=list(message.data)
        record('result_received',data=data)
        item=gate.take(data)
        if item is None:
            record('result_ignored',data=data)
            return
        business_callback(receiver,message)
        if not receiver.board2_received or receiver.board2_result!=data:
            record('business_rejected',data=data)
            gate.finish()
            return
        last['result']=list(receiver.board2_result)
        record('business_received',data=last['result'])
        thread=threading.Thread(target=speak,args=(item[0],item[1],data))
        # Let the interpreter wait for play_audio's shutdown cleanup to end play.
        thread.daemon=False
        thread.start()
    def status(request):
        return TriggerResponse(True,json.dumps(dict(last,visit=gate.visit,speaking=gate.speaking,armed=gate.armed,used=gate.used)))
    rospy.Subscriber('/pharmacy_vision_debug/board2_return',Int32MultiArray,result,queue_size=10)
    rospy.Service('/pharmacy_vision_debug/rearm',Trigger,rearm)
    rospy.Service('/pharmacy_vision_debug/status',Trigger,status)
    rearm(None)
    rospy.spin()


if __name__=='__main__':main()
