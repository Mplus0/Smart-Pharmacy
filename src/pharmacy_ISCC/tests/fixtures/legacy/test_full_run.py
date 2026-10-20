#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Native ROS message types; all action clients and ROS operations are fakes."""
from __future__ import print_function
import os
import sys
import math
import unittest
import itertools
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'scripts'))
sys.path.append('/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts')
import F1_yaofang_v5 as main
from nav_msgs.msg import OccupancyGrid
from waypoint_console import arrival_check, target_pose, quaternion_yaw


class FakeROS(object):
    Time = type('Time', (), {'now': staticmethod(lambda: main.rospy_original.Time(1))})
    Duration = staticmethod(lambda v: v)
    def __init__(self):
        self.params = {}
        self.stops = []
    def get_param(self, name, default): return self.params.get(name, default)
    def signal_shutdown(self, reason): self.stops.append(reason)
    def sleep(self, value): pass
    def loginfo(self, *args): pass
    def logwarn(self, *args): pass
    def logerr(self, *args): pass
    def loginfo_throttle(self, *args): pass
    def is_shutdown(self): return bool(self.stops)


class Pub(object):
    def __init__(self): self.messages = []
    def publish(self, value): self.messages.append(value)


class Client(object):
    def __init__(self, status=3, finished=True):
        self.status, self.finished = status, finished
        self.sent = []; self.canceled = 0
    def send_goal(self, goal): self.sent.append(goal)
    def wait_for_result(self, duration): return self.finished
    def get_state(self): return self.status
    def cancel_goal(self): self.canceled += 1


class Guard(object):
    def __init__(self, dx=0, angle=0): self.dx, self.angle = dx, angle
    def grid(self):
        g = OccupancyGrid(); g.info.width = 400; g.info.height = 400
        g.info.resolution = .1; g.info.origin.position.x = -10
        g.info.origin.position.y = -10; g.info.origin.orientation.w = 1
        g.data = [0]*160000
        return g, {}
    def pose(self): return target_pose(0, 0, 0)
    def stopped_pose(self, requested):
        actual = target_pose(requested['x']+self.dx, requested['y'], requested['yaw_deg']+self.angle)
        check = arrival_check(actual, requested, .05, 3)
        check['stable_sec'] = 1
        return actual, check


class FullRunTests(unittest.TestCase):
    def setUp(self):
        main.rospy_original = main.rospy
        self.ros = FakeROS(); main.rospy = self.ros
    def tearDown(self): main.rospy = main.rospy_original
    def app(self):
        a = main.MoveBaseSquare.__new__(main.MoveBaseSquare)
        a.init_runtime_state(); a.waypoints = a.build_waypoints()
        for key in ['pub_referee_task', 'pub_referee_cv1', 'pub_referee_cv2', 'pub_nav_state', 'pub_round_done']:
            setattr(a, key, Pub())
        a.clear_costmaps_service = lambda: None
        a.play_audio = lambda name: None
        a.arrival_guard = Guard(); a.move_base = Client()
        return a
    def test_ten_coordinates_and_headings(self):
        expected = [(1.4,1.9,90),(.6,2.4,90),(1.4,2.8,90),(-1.6,.9,-90),
                    (-2.4,1.5,-90),(-1.6,1.7,-90),(-2.4,2.4,-90),
                    (0,0,0),(-1.7,3.7,180),(.65,0,0)]
        for p, (x,y,yaw) in zip(self.app().waypoints, expected):
            self.assertAlmostEqual(p.position.x, x); self.assertAlmostEqual(p.position.y, y)
            q=p.orientation
            diff = math.degrees(quaternion_yaw([q.x,q.y,q.z,q.w]))-yaw
            self.assertAlmostEqual(math.sin(math.radians(diff)/2), 0)
        self.assertEqual(main.COMMON['board2']['force_label_for_debug'], 'free')
        self.assertTrue(main.COMMON['board1']['require_four_windows'])
    def test_single_car_default_and_dual_opt_in(self):
        a=self.app(); self.assertFalse(a.enable_dual_car); self.assertTrue(a.have_turn)
        self.ros.params['~enable_dual_car']=True
        self.assertTrue(self.app().enable_dual_car)
    def test_all_28_pickup_and_destination_combinations_return_home(self):
        for bits in itertools.product([0,1], repeat=3):
            if not any(bits): continue
            for lab in range(4):
                self.ros.stops=[]; a=self.app(); visited=[]
                def move(goal):
                    p=goal.target_pose.pose.position; visited.append((p.x,p.y)); return True
                a.move=move
                def board1():
                    a.windows_A,a.windows_B,a.windows_C=bits
                    a.windows_count=sum(bits); a.windows_1234=lab
                    return True
                a.wait_for_board1_result=board1
                a.wait_for_board2_result=lambda: True
                a.handle_go_to_board1(); a.handle_go_to_pickup_windows()
                a.handle_go_to_board2(); a.handle_go_to_lab_window(); a.handle_go_back_home()
                pickups=[(1.4,1.9),(.6,2.4),(1.4,2.8)]
                expected=[(.65,0)] + [p for p,yes in zip(pickups,[bits[2],bits[0],bits[1]]) if yes]
                expected += [(-1.7,3.7),[(-2.4,2.4),(-1.6,1.7),(-2.4,1.5),(-1.6,.9)][lab],(0,0)]
                self.assertEqual(visited, expected)
                self.assertEqual(a.completed_rounds,1)
                self.assertEqual(self.ros.stops,['Full run complete at home'])
                self.assertEqual(a.pub_round_done.messages,[])
    def test_success_continues_without_post_goal_pose_check(self):
        for dx,angle in [(.0548,.39),(.1,10)]:
            a=self.app(); g=a.make_goal(a.waypoints[0])
            a.arrival_guard=Guard(dx,angle)
            def forbidden(*args):
                raise AssertionError('No post-goal pose/stopping checks allowed')
            a.arrival_guard.stopped_pose=forbidden
            # Pre-goal localization must be available; querying it again after
            # the goal completes must not be necessary for SUCCEEDED.
            a.arrival_guard.pose=lambda: None
            original_send=a.move_base.send_goal
            def send(goal):
                a.arrival_guard.pose=forbidden
                a.arrival_guard.grid=forbidden
                original_send(goal)
            a.move_base.send_goal=send
            self.assertTrue(a.move(g))
            self.assertEqual(len(a.move_base.sent),1)
            self.assertEqual(self.ros.stops,[])
    def test_navigation_abort_and_timeout_stop_run(self):
        for status,finished in [(4,True),(3,False)]:
            self.ros.stops=[]; a=self.app(); a.move_base=Client(status,finished)
            self.assertFalse(a.move(a.make_goal(a.waypoints[0])))
            self.assertTrue(self.ros.stops)
    def test_invalid_target_stops_before_goal(self):
        a=self.app(); goal=a.make_goal(a.waypoints[0]);goal.target_pose.pose.position.x=100
        self.assertFalse(a.move(goal));self.assertEqual(a.move_base.sent,[])


if __name__ == '__main__': unittest.main(verbosity=2)
