#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""单航点导航测试工具。

从 waypoints.yaml 读取航点，调用 move_base 前往指定目标。
用于逐个调试停靠点位置是否准确。

用法:
  # 前往识别板一
  rosrun pharmacy_mplus0_debug waypoint_tester.py _target:=board1

  # 前往体检窗口 A
  rosrun pharmacy_mplus0_debug waypoint_tester.py _target:=exam_A

  # 前往化验窗口 1
  rosrun pharmacy_mplus0_debug waypoint_tester.py _target:=lab_1
"""

import os
import sys
import rospy
import rospkg
import yaml
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf.transformations


class WaypointTester(object):
    """单航点导航测试器。"""

    def __init__(self):
        rospy.init_node("waypoint_tester", anonymous=True)

        target = rospy.get_param("~target", "start")
        clear_costmaps = rospy.get_param("~clear", True)

        # 加载 waypoints.yaml。
        waypoints = self._load_waypoints()
        self._waypoints = waypoints.get("waypoints", {})
        self._map_frame = waypoints.get("frames", {}).get("map", "map")

        if target not in self._waypoints:
            rospy.logerr(
                "航点 '%s' 不在 waypoints.yaml 中。可用: %s",
                target, list(self._waypoints.keys()),
            )
            sys.exit(1)

        wp = self._waypoints[target]
        rospy.loginfo(
            "目标: %s → (%.3f, %.3f, yaw=%.4f)",
            target, wp["x"], wp["y"], wp.get("yaw", 0.0),
        )

        # 初始化 move_base 客户端。
        self._client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        rospy.loginfo("等待 move_base action server...")
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("move_base 未就绪，请先启动导航")
            sys.exit(1)

        # 构建目标。
        goal = self._build_goal(wp)

        if clear_costmaps:
            self._clear_costmaps()

        rospy.loginfo("开始导航到 %s ...", target)
        self._client.send_goal(goal)
        finished = self._client.wait_for_result(
            rospy.Duration(60.0)
        )
        if not finished:
            rospy.logwarn("导航超时")
            self._client.cancel_goal()
        else:
            state = self._client.get_state()
            if state == 3:  # SUCCEEDED
                rospy.loginfo("成功到达 %s", target)
            else:
                rospy.logwarn("导航结束但未成功，状态码: %d", state)

    def _build_goal(self, wp):
        """构建 MoveBaseGoal。"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self._map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = wp["x"]
        goal.target_pose.pose.position.y = wp["y"]
        goal.target_pose.pose.position.z = 0.0

        yaw = wp.get("yaw", 0.0)
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
        return goal

    @staticmethod
    def _clear_costmaps():
        """清理 costmap。"""
        from std_srvs.srv import Empty
        try:
            rospy.wait_for_service(
                "/move_base/clear_costmaps", timeout=3.0
            )
            srv = rospy.ServiceProxy(
                "/move_base/clear_costmaps", Empty
            )
            srv()
            rospy.loginfo("costmap 已清理")
        except Exception as exc:
            rospy.logwarn("清理 costmap 失败: %s", exc)

    @staticmethod
    def _load_waypoints():
        """加载 waypoints.yaml。"""
        try:
            ros_pack = rospkg.RosPack()
            pkg_path = ros_pack.get_path("pharmacy_mplus0")
            path = os.path.join(pkg_path, "config", "waypoints.yaml")
            with open(path, "r") as fh:
                return yaml.safe_load(fh) or {}
        except IOError:
            return {}


def main():
    try:
        WaypointTester()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
