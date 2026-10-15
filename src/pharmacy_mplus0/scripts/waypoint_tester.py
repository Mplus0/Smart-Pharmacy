#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
waypoint_tester.py

Standalone waypoint test node for pharmacy_mplus0.

Usage examples:
  rosrun pharmacy_mplus0 waypoint_tester.py _target:=A
  rosrun pharmacy_mplus0 waypoint_tester.py _target:=board1
  rosrun pharmacy_mplus0 waypoint_tester.py _target:=lab3
  rosrun pharmacy_mplus0 waypoint_tester.py _target:=home

Supported targets:
  A, B, C, lab1, lab2, lab3, lab4, board1, board2, home
"""

import sys

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler

from pharmacy_mplus0.config import COMMON, get_car_config, get_car_id


STATUS_TEXT = {
    GoalStatus.PENDING: "PENDING",
    GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED",
    GoalStatus.PREEMPTING: "PREEMPTING",
    GoalStatus.RECALLING: "RECALLING",
    GoalStatus.RECALLED: "RECALLED",
    GoalStatus.LOST: "LOST",
}


def make_pose_from_tuple(item):
    """Create geometry_msgs/Pose from (x, y, q_index)."""
    x, y, q_index = item
    angles = COMMON["nav"]["euler_angles"]
    yaw = angles[int(q_index)]
    q = quaternion_from_euler(0, 0, yaw, axes="sxyz")
    return Pose(Point(float(x), float(y), 0.0), Quaternion(*q))


def get_waypoint_tuple(target, car_config):
    """Return (x, y, q_index) for target."""
    target = target.strip()
    wp = COMMON["nav"]["waypoints"]

    if target in wp:
        return wp[target]

    if target == "home":
        return car_config["home_pose"]

    raise ValueError(
        "Unknown target: %s. Supported: A, B, C, lab1, lab2, lab3, lab4, board1, board2, home"
        % target
    )


def clear_costmaps(service_name, timeout_sec):
    """Clear move_base costmaps if service exists."""
    try:
        rospy.loginfo("Waiting for clear costmaps service: %s", service_name)
        rospy.wait_for_service(service_name, timeout=timeout_sec)
        srv = rospy.ServiceProxy(service_name, Empty)
        srv()
        rospy.loginfo("Costmaps cleared.")
        return True
    except Exception as e:
        rospy.logwarn("Clear costmaps failed or timed out: %s", str(e))
        return False


def main():
    rospy.init_node("test_move_to_waypoint", anonymous=False)

    car_id = get_car_id(default=1)
    car_config = get_car_config(car_id)

    target = rospy.get_param("~target", "A")
    timeout_sec = float(rospy.get_param("~timeout_sec", 60.0))
    wait_server_sec = float(rospy.get_param("~wait_server_sec", 30.0))
    clear_before_move = bool(rospy.get_param("~clear_costmaps", True))
    dry_run = bool(rospy.get_param("~dry_run", False))

    move_base_name = COMMON["topics"]["move_base_action"]
    clear_service = COMMON["topics"]["clear_costmaps_service"]

    try:
        wp_tuple = get_waypoint_tuple(target, car_config)
    except Exception as e:
        rospy.logerr(str(e))
        sys.exit(1)

    pose = make_pose_from_tuple(wp_tuple)

    rospy.logwarn("========================================")
    rospy.logwarn("Waypoint test")
    rospy.logwarn("CAR_ID: %s", car_id)
    rospy.logwarn("target: %s", target)
    rospy.logwarn("x: %.3f, y: %.3f, q_index: %s", wp_tuple[0], wp_tuple[1], wp_tuple[2])
    rospy.logwarn("timeout_sec: %.1f", timeout_sec)
    rospy.logwarn("dry_run: %s", dry_run)
    rospy.logwarn("========================================")

    if dry_run:
        rospy.logwarn("Dry run enabled. Goal will not be sent.")
        return

    client = actionlib.SimpleActionClient(move_base_name, MoveBaseAction)

    rospy.loginfo("Waiting for move_base action server: %s", move_base_name)
    if not client.wait_for_server(rospy.Duration(wait_server_sec)):
        rospy.logerr("move_base action server not available after %.1f seconds.", wait_server_sec)
        sys.exit(2)

    if clear_before_move:
        clear_costmaps(clear_service, 5.0)
        rospy.sleep(0.5)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = pose

    rospy.logwarn("Sending goal to move_base: %s", target)
    client.send_goal(goal)

    finished = client.wait_for_result(rospy.Duration(timeout_sec))
    if not finished:
        rospy.logerr("Move timeout after %.1f seconds. Canceling goal.", timeout_sec)
        client.cancel_goal()
        sys.exit(3)

    state = client.get_state()
    state_text = STATUS_TEXT.get(state, str(state))

    if state == GoalStatus.SUCCEEDED:
        rospy.logwarn("Goal reached successfully: %s", target)
        sys.exit(0)

    rospy.logerr("Goal failed: target=%s, state=%s", target, state_text)
    sys.exit(4)


if __name__ == "__main__":
    main()
