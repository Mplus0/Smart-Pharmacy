# -*- coding: utf-8 -*-
"""move_base 导航封装。

本模块为 move_base SimpleActionClient 提供了一层语义化封装。
主控不需要直接拼接 MoveBaseGoal、处理 action 状态机或
清理 costmap——只需要调用 go_to("exam_A")。

所有坐标集中放在 waypoints.yaml，导航超时从 strategy.yaml 读取，
避免在主控代码中硬编码坐标和数字。
"""

import os
import threading

import rospy
import rospkg
import yaml
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
import tf.transformations

from pharmacy_mplus0.constants import (
    NAV_ACTION_NAME,
    NAV_CLEAR_COSTMAPS_SRV,
    NAV_DEFAULT_TIMEOUT_SEC,
)

# 未提供包路径时使用的默认配置文件名，用于从包目录下 locate。
DEFAULT_WAYPOINTS_FILE = "waypoints.yaml"
DEFAULT_STRATEGY_FILE = "strategy.yaml"


class NavigationClient(object):
    """封装 move_base 导航，对外暴露逻辑名称（如 exam_A / lab_1）。"""

    def __init__(self, waypoints_path=None, strategy_path=None, dry_run=False):
        """初始化导航客户端。

        waypoints_path / strategy_path 支持外部注入，便于单元测试。
        未指定时通过 rospkg 从 pharmacy_mplus0/config 目录加载。

        dry_run=True 时跳过 move_base，go_to() 始终返回 True，
        用于无导航的调试场景（如 test_full_dryrun.launch）。
        """
        # 加载航点配置 (waypoints.yaml)
        if waypoints_path is None:
            waypoints_path = self._resolve_config_path(DEFAULT_WAYPOINTS_FILE)
        self._waypoints_config = self._load_yaml(waypoints_path)
        self._waypoints = self._waypoints_config.get("waypoints", {})
        self._map_frame = self._waypoints_config.get("frames", {}).get(
            "map", "map"
        )

        # 加载策略配置 (strategy.yaml)，提取导航默认超时
        if strategy_path is None:
            strategy_path = self._resolve_config_path(DEFAULT_STRATEGY_FILE)
        strategy_config = self._load_yaml(strategy_path)
        self._nav_timeout = (
            strategy_config.get("timeouts", {}).get(
                "nav_default_seconds", NAV_DEFAULT_TIMEOUT_SEC
            )
        )

        # 干跑模式：跳过 move_base，go_to 始终返回 True。
        self._dry_run = dry_run

        # 创建 move_base action 客户端。
        # 注意：SimpleActionClient 创建时不等待服务器，由 _ensure_server 按需等待。
        self._client = actionlib.SimpleActionClient(
            NAV_ACTION_NAME, MoveBaseAction
        )
        self._server_connected = False

        # 锁，保证一个时刻只有一个导航请求在处理。
        self._lock = threading.Lock()

        rospy.loginfo(
            "[NavigationClient] 已加载 %d 个航点，默认超时 %.1f 秒",
            len(self._waypoints),
            self._nav_timeout,
        )

    # ----- 对外主接口 --------------------------------------------------

    def go_to(self, waypoint_name, timeout_sec=None):
        """发送导航目标并阻塞等待结果。

        参数:
            waypoint_name: waypoints.yaml 中的逻辑名称（如 exam_A / lab_1 / board1）。
            timeout_sec:   本次导航超时秒数，None 则使用 strategy.yaml 中的
                           timeouts.nav_default_seconds。

        返回:
            True  导航成功（move_base 返回 SUCCEEDED，或干跑模式）。
            False 导航失败（超时、取消、被抢占、中止或目标点不存在）。
        """
        if timeout_sec is None:
            timeout_sec = self._nav_timeout

        # 干跑模式：跳过真实导航，直接返回成功。
        if self._dry_run:
            rospy.loginfo(
                "[NavigationClient] [DRY-RUN] 跳过导航 %s -> 模拟成功",
                waypoint_name,
            )
            return True

        goal = self._build_goal(waypoint_name)
        if goal is None:
            rospy.logerr("[NavigationClient] 未知航点: %s", waypoint_name)
            return False

        with self._lock:
            self._ensure_server()
            rospy.loginfo(
                "[NavigationClient] 正在前往 %s，超时 %.1f 秒",
                waypoint_name,
                timeout_sec,
            )
            self._client.send_goal(goal)
            finished = self._client.wait_for_result(
                timeout=rospy.Duration(timeout_sec)
            )
            if not finished:
                # 超时后取消当前目标，让 move_base 回到空闲状态。
                self._client.cancel_goal()
                rospy.logwarn(
                    "[NavigationClient] 导航至 %s 超时 (%.1f 秒)",
                    waypoint_name,
                    timeout_sec,
                )
                return False

            state = self._client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo(
                    "[NavigationClient] 成功到达 %s", waypoint_name
                )
                return True

            rospy.logwarn(
                "[NavigationClient] 导航至 %s 失败，状态码 %d",
                waypoint_name,
                state,
            )
            return False

    def clear_costmaps(self):
        """调用 /move_base/clear_costmaps 服务清理代价地图。

        每段导航结束后调用，降低雷达噪点或残留障碍对后续规划的影响。
        服务不可用时仅打印警告，不抛异常，不阻塞主控。
        """
        try:
            rospy.wait_for_service(
                NAV_CLEAR_COSTMAPS_SRV, timeout=2.0
            )
            clear_srv = rospy.ServiceProxy(
                NAV_CLEAR_COSTMAPS_SRV, Empty
            )
            clear_srv()
            rospy.logdebug("[NavigationClient] costmap 已清理")
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn(
                "[NavigationClient] 清理 costmap 失败: %s", exc
            )

    def cancel(self):
        """取消当前导航目标（不等结果）。"""
        with self._lock:
            self._client.cancel_goal()
            rospy.loginfo("[NavigationClient] 已取消当前导航目标")

    def get_waypoint(self, name):
        """返回航点的 (x, y, yaw) 元组，未找到时返回 None。

        主控可以用它预先校验目标是否存在。
        """
        wp = self._waypoints.get(name)
        if wp is None:
            return None
        return (float(wp["x"]), float(wp["y"]), float(wp["yaw"]))

    # ----- 内部方法 ----------------------------------------------------

    def _ensure_server(self):
        """确保 move_base action 服务器可达，最多等待 5 秒。"""
        if self._server_connected:
            return
        if not self._client.wait_for_server(rospy.Duration(5.0)):
            raise rospy.ROSException(
                "move_base action 服务器未就绪，无法导航"
            )
        self._server_connected = True

    def _build_goal(self, waypoint_name):
        """根据航点名称构造 MoveBaseGoal。

        从 waypoints.yaml 读取坐标和朝向，构造 map 坐标系下的目标位姿。
        航点不存在时返回 None。
        """
        coords = self.get_waypoint(waypoint_name)
        if coords is None:
            return None

        x, y, yaw = coords
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self._map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        # 将 yaw（偏航角）转换为四元数，保证 move_base 能正确理解朝向。
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        return goal

    @staticmethod
    def _resolve_config_path(filename):
        """通过 rospkg 获取包内 config/<filename> 的绝对路径。

        这样主控可以在任何工作目录启动，不依赖相对路径。
        """
        ros_pack = rospkg.RosPack()
        pkg_path = ros_pack.get_path("pharmacy_mplus0")
        return os.path.join(pkg_path, "config", filename)

    @staticmethod
    def _load_yaml(path):
        """读取 YAML 文件，返回 dict。文件不存在时返回空 dict。"""
        try:
            with open(path, "r") as fh:
                data = yaml.safe_load(fh)
            return data if data else {}
        except IOError:
            rospy.logwarn(
                "[NavigationClient] 无法读取配置: %s", path
            )
            return {}
