#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Read camera frames for three static trials; publish no business/velocity topics."""
from __future__ import print_function
import argparse
import hashlib
import imp
import json
import os
import sys
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from test_board1_recognition import SCRIPTS, EXPECTED_TEXT, EXPECTED_MSG, Recorder


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='/tmp/board1_live_verification/results.json')
    args = parser.parse_args(rospy.myargv()[1:])
    output = os.path.dirname(os.path.abspath(args.output))
    if not os.path.isdir(output):
        os.makedirs(output)
    rospy.init_node('board1_static_verification', anonymous=True, disable_signals=True)
    imp.load_source('dual_car_config', os.path.join(SCRIPTS, 'dual_car_config.py'))
    m = imp.load_source('board1_live_candidate', os.path.join(SCRIPTS, 'F1_detect_code_v5.py'))
    for name in ['loginfo', 'logwarn', 'loginfo_throttle', 'logwarn_throttle']:
        setattr(rospy, name, lambda *a, **k: None)
    m.pub_flag = Recorder()
    m.pub_board1_all_text = Recorder()
    bridge = CvBridge()
    original_find, original_stable = m.find_board1_window_boxes, m.stable_publish_board1
    observed = {'boxes': None, 'all_text': None}

    def find(frame):
        observed['boxes'] = original_find(frame)
        return observed['boxes']

    def stable(text):
        observed['all_text'] = list(text)
        return original_stable(text)

    m.find_board1_window_boxes = find
    m.stable_publish_board1 = stable
    report = {'expected_all_text': EXPECTED_TEXT, 'expected_message': EXPECTED_MSG,
              'rows': [], 'rounds': [], 'success': False}
    hashes = set()
    try:
        assert m.BOARD1_REQUIRE_FOUR_WINDOWS
        assert not m.BOARD1_WINDOW_COMPLETE_FROM_THREE
        for visit in range(3):
            m.nav_state_cb(Int32(data=9))
            m.nav_state_cb(Int32(data=10))
            started = time.time()
            count_before = len(m.pub_flag.messages)
            for attempt in range(10):
                msg = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=6)
                frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                sha = hashlib.sha256(frame.tostring()).hexdigest()
                assert sha not in hashes, 'Repeated identical input frame in live verification'
                hashes.add(sha)
                if attempt == 0:
                    cv2.imwrite(os.path.join(output, 'round_%d.png' % (visit + 1)), frame)
                observed.update(boxes=None, all_text=None)
                tick = time.time()
                m.detect_and_publish_board1(frame)
                boxes = observed['boxes']
                report['rows'].append({
                    'visit': visit + 1, 'attempt': attempt + 1, 'frame_sha256': sha,
                    'camera_stamp': [msg.header.stamp.secs, msg.header.stamp.nsecs],
                    'all_text': observed['all_text'],
                    'real_window_count': sum(not it.get('inferred', False) for it in boxes) if boxes else 0,
                    'processing_ms': round((time.time() - tick) * 1000, 1)})
                if len(m.pub_flag.messages) > count_before:
                    assert m.pub_flag.messages[-1] == EXPECTED_MSG, m.pub_flag.messages[-1]
                    report['rounds'].append({'visit': visit + 1, 'frames': attempt + 1,
                                             'latency_sec': round(time.time() - started, 3),
                                             'message': m.pub_flag.messages[-1]})
                    break
                time.sleep(0.15)
            else:
                raise AssertionError('No stable result after ten frames in visit %d' % (visit + 1))
        assert len(m.pub_flag.messages) == 3
        report['success'] = True
    except Exception as error:
        report['error'] = repr(error)
    finally:
        report['published_messages'] = m.pub_flag.messages
        report['unique_input_frames'] = len(hashes)
        with open(args.output, 'w') as stream:
            json.dump(report, stream, ensure_ascii=True, indent=2)
    print(json.dumps(report, ensure_ascii=True))
    return 0 if report['success'] else 1


if __name__ == '__main__':
    sys.exit(main())
