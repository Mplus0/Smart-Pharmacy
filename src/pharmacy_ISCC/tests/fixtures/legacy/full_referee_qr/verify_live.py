#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Validate an already projected board against fresh production ROS outputs."""
from __future__ import print_function
import argparse
import hashlib
import imp
import json
import os
import sys
import threading
import time
import cv2
import rospy
import rosgraph
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Int32MultiArray, String

CASES = {item['case']: (item['all_text'], item['message']) for item in json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'manifest.json')))['scenes']}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', choices=sorted(CASES))
    parser.add_argument('--rounds', type=int, default=3)
    args = parser.parse_args(rospy.myargv()[1:])
    expected_text, expected_msg = CASES[args.case]
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'live_results', args.case)
    if not os.path.isdir(root):
        os.makedirs(root)
    rospy.init_node('board1_projection_verification', anonymous=True, disable_signals=True)
    publishers, subscribers, services = rosgraph.Master(rospy.get_name()).getSystemState()
    state_readers = dict(subscribers).get('/nav_state', [])
    assert state_readers == ['/board1_debug_detector'], state_readers
    assert not any('nav_pharmacy' in n for _, nodes in publishers + subscribers for n in nodes)
    sys.path.insert(0, '/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts')
    m = imp.load_source('board1_projection_readonly', '/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/F1_detect_code_v5.py')
    bridge = CvBridge()
    lock = threading.Lock()
    received = {'cam': [], 'all': []}
    def cam(msg):
        with lock:
            received['cam'].append(list(msg.data))
    def all_text(msg):
        with lock:
            received['all'].append(json.loads(msg.data))
    sub1 = rospy.Subscriber('/cam_return', Int32MultiArray, cam, queue_size=10)
    sub2 = rospy.Subscriber('/board1_all_text', String, all_text, queue_size=10)
    pub = rospy.Publisher('/nav_state', Int32, queue_size=5)
    report = {'case': args.case, 'expected_all_text': expected_text, 'expected_message': expected_msg,
              'nav_state_subscribers': state_readers, 'rounds': [], 'success': False}
    try:
        deadline = time.time() + 5
        while pub.get_num_connections() != 1 and time.time() < deadline:
            time.sleep(.1)
        assert pub.get_num_connections() == 1
        time.sleep(.5)
        for visit in range(args.rounds):
            pub.publish(Int32(data=9)); time.sleep(.3)
            with lock:
                prior_seq = max([int(v.get('seq', 0)) for v in received['all']] or [0])
                received['cam'] = []; received['all'] = []
            camera_msg = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=6)
            frame = bridge.imgmsg_to_cv2(camera_msg, desired_encoding='bgr8')
            cv2.imwrite(root + '/round_%d.png' % (visit + 1), frame)
            boxes = m.find_board1_window_boxes(frame)
            decoded = m.decode_board1_by_windows(frame)
            row = {'visit': visit + 1, 'prior_seq': prior_seq, 'frame_sha256': hashlib.sha256(frame.tostring()).hexdigest(),
                   'real_window_count': sum(not b.get('inferred', False) for b in boxes) if boxes else 0,
                   'snapshot_decoded': decoded}
            report['rounds'].append(row)
            if expected_text is not None:
                assert decoded == expected_text, row
                assert row['real_window_count'] == 4, row
            else:
                assert row['real_window_count'] < 4 and decoded is None, row
            started = time.time(); pub.publish(Int32(data=10))
            deadline = started + (8 if expected_msg is not None else 5)
            while time.time() < deadline:
                with lock:
                    new_all = [v for v in received['all'] if int(v.get('seq', 0)) > prior_seq]
                    messages = list(received['cam'])
                if expected_msg is not None and new_all and messages:
                    break
                time.sleep(.05)
            pub.publish(Int32(data=9)); time.sleep(.4)
            with lock:
                new_all = [v for v in received['all'] if int(v.get('seq', 0)) > prior_seq]
                messages = list(received['cam'])
            row.update(latency_sec=round(time.time() - started - .4, 3), messages=messages, all_text_messages=new_all)
            if expected_msg is None:
                assert not messages and not new_all, row
            else:
                assert messages == [expected_msg], row
                # Production intentionally transmits the same sequence three times.
                assert len(new_all) == 3 and len(set(v['seq'] for v in new_all)) == 1, row
                assert all(v['all_text'] == expected_text and v['selected_index'] == expected_msg[4]
                           and v['selected_msg'] == expected_msg for v in new_all), row
        report['success'] = True
    except Exception as error:
        report['error'] = repr(error)
    finally:
        pub.publish(Int32(data=9))
        with open(root + '/results.json', 'w') as stream:
            json.dump(report, stream, indent=2)
    print(json.dumps(report))
    return 0 if report['success'] else 1

if __name__ == '__main__':
    sys.exit(main())
