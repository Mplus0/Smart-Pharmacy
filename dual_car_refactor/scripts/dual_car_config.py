#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
dual_car_config.py

双车系统集中配置文件。
用法：同一套 v5/v3/v2 脚本可同时给 car1 / car2 使用，启动前设置环境变量：
    export CAR_ID=1   # 一号车
    export CAR_ID=2   # 二号车

现场只需要优先修改 CARS 与 COMMON 中的参数，不建议再到各节点脚本里改硬编码值。
"""
import os


def get_car_id(default=1):
    """从环境变量 CAR_ID 获取车号；没有设置时使用 default。"""
    value = os.environ.get("CAR_ID", str(default))
    try:
        car_id = int(value)
    except Exception:
        raise ValueError("CAR_ID 必须是整数，当前为: %r" % value)
    if car_id not in CARS:
        raise ValueError("CAR_ID 只支持 %s，当前为: %s" % (sorted(CARS.keys()), car_id))
    return car_id


def get_car_config(car_id=None, default=1):
    """获取指定车号配置。"""
    if car_id is None:
        car_id = get_car_id(default=default)
    return CARS[int(car_id)]


COMMON = {
    "paths": {
        "audio_dir": "/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian",
        "board2_template_dir": "/home/EPRobot/robot_ws/src/pharmacy_pkg/pictures/board_2",
    },

    "topics": {
        "cmd_vel": "/cmd_vel",
        "cam_return": "/cam_return",
        "board2_return": "/board2_return",
        "nav_state": "/nav_state",
        "referee_task": "/referee_task",
        "referee_cv1": "/referee_cv1",
        "referee_cv2": "/referee_cv2",
        "round_done": "/dual_car/round_done",
        "peer_done": "/dual_car/peer_done",
        "odom": "/odometry/filtered",
        "clear_costmaps_service": "/move_base/clear_costmaps",
        "move_base_action": "move_base",
    },

    "states": {
        "WAIT_TURN": 8,
        "GO_TO_BOARD1": 9,
        "BOARD1_RECOGNIZING": 10,
        "GO_TO_PICKUP_WINDOWS": 11,
        "GO_TO_BOARD2": 12,
        "BOARD2_RECOGNIZING": 13,
        "GO_TO_LAB_WINDOW": 14,
        "GO_BACK_HOME": 15,
    },

    "board1": {
        "approx_poly_dp_epsilon": 0.02,
        "wh_rate": 0.4,
        "min_area": 500,
        "max_area": 20000,
        "min_center_distance": 20,
        "locator_min": 44,
        "locator_max": 60,
        "result_min": 20,
        "stable_frames": 3,
        "publish_all_text": True,
        "share_all_text_topic": "/board1_all_text",
        "peer_all_text_topic": "/dual_car/peer_board1_all_text",

        # v2识别策略：先找4个大窗口框，再逐窗口轻量解码；二维码数量允许为0~4。
        "window_detect_enable": True,
        "window_min_area": 2500,
        "window_max_area": 80000,
        "window_square_wh_rate": 0.45,
        "window_dedup_distance": 55,
        "window_inner_margin_rate": 0.16,
        "window_warp_size": 240,
        "window_complete_from_three": True,

        # 直接扫整图二维码只作为兜底：必须看到4个二维码才排序，少于4个不猜窗口。
        "direct_decode_enable": False,
        "direct_min_qr_count": 4,
        "direct_dedup_distance": 45,
    },

    "board2": {
        "match_threshold": 0.68,
        "stable_frames": 2,
        "score_margin": 0.08,
        "publish_interval": 0.5,
        "roi": (0.0, 0.0, 1.0, 1.0),
        "template_scales": [0.85, 0.95, 1.0, 1.05, 1.15],
        # 调试时可改成 "busy_5" / "free" 等；正式比赛必须为 None，才会启用真实模板匹配。
        "force_label_for_debug": "free",
        "force_score_for_debug": 1.0,
    },

    "detect": {
        "frame_rotate_angle": 0,
        # 板一识别阶段提高处理频率；ROS topic 模式下不会再依赖 HTTP 旧帧丢弃。
        "drop_frame_count": 2,
        "limit_rate_hz": 6,
        "idle_grab_sleep_sec": 0.1,
        "camera_reconnect_sleep_sec": 1.0,

        # 图像输入源：默认直接订阅 ROS Image，HTTP 视频流仅作为兜底。
        # 可选："ros_topic" / "http" / "ros_compressed"。
        "image_source": "ros_topic",
        "camera_topic": "/camera/rgb/image_raw",
        "camera_fallback_to_http": True, # 是否允许 ROS 图像源异常时自动切换到 HTTP 视频流；调试时可改 False 强制使用 HTTP。
        "camera_max_age_sec": 0.5,
        "ros_image_buff_size": 2 ** 24,
    },

    "nav": {
        "state_machine_rate_hz": 5,
        "wait_result_rate_hz": 10,
        "move_timeout_sec": 30,
        # 0 表示沿用旧逻辑：一直等待。建议调试期设 20~60，避免识别节点异常时状态机永久卡死。
        "board1_wait_timeout_sec": 0,
        "board2_wait_timeout_sec": 0,
        "move_base_wait_server_sec": 60,
        "pickup_task_hold_sec": 1.1,
        "lab_task_hold_sec": 1.1,
        "post_clear_costmap_sleep_sec": 0.5,
        "home_arrive_sleep_sec": 0.1,
        "cv_publish_repeat": 3,
        "cv_publish_interval_sec": 0.05,
        "dual_publish_repeat": 1,
        "dual_publish_interval_sec": 0.10,
        "euler_angles": [
            1.5707963267948966, 1.5707963267948966, 1.5707963267948966,
            -1.5707963267948966, -1.5707963267948966, -1.5707963267948966, -1.5707963267948966,
            0.0, -3.141592653589793, 0.0,
        ],
        # waypoint 顺序必须保持旧代码约定：0=C,1=A,2=B,3=4号,4=3号,5=2号,6=1号,7=起点,8=板2,9=板1
        "waypoints": {
            "C": (1.410, 1.828, 0),  # 0.48，2.17
            "A": (0.685, 2.450, 8),  # -0.47,2.19
            "B": (1.385, 2.852, 2),  # -0.13，2.97
            "lab4": (-1.000, 0.800, 3),  # -0.9，0.04
            "lab3": (-1.800, 1.240, 4),  # -1.71，-0.12
            "lab2": (-1.100, 1.600, 7),  # -1.48，0.55
            "lab1": (-1.800, 2.300, 6),  # -2.29，0.74
            "board2": (-0.454, 3.791, 8),
            "board1": (0.591, -0.269, 9),
        },
    },

    "referee": {
        "server_ip": "192.168.124.2",
        "server_port": 8888,
        "send_rate_hz": 2,
        "socket_timeout_sec": 3.0,
        "reconnect_sleep_sec": 0.5,
        "valid_tasks": ["R", "A", "B", "C", "1", "2", "3", "4"],
    },

    "dual_tcp": {
        "local_host": "0.0.0.0",
        "socket_timeout_sec": 2.0,
        "retry_interval_sec": 0.20,
        "send_duration_sec": 3.0,
        "peer_done_publish_repeat": 3,
        "peer_done_publish_interval_sec": 0.05,
        "server_accept_timeout_sec": 1.0,
        "client_recv_timeout_sec": 2.0,
        "max_line_chars": 2048,
        # None 表示不校验口令；如现场网络不可信，可设置同一个字符串，例如 "race-secret"。
        "shared_token": None, # 作用是在双方都设置了相同字符串时，TCP 消息里会携带这个口令，接收方验证不通过就丢弃消息；调试时可改 None 以简化流程，但正式比赛建议设置以防误连。
        # True 时只接受 PEER_IP 发来的 done。若现场 IP 会漂移，可临时改 False。
        "check_peer_ip": True,
    },

    "lab_info": {
        3: {"real_window": 4, "sample_name": "血浆", "pickup_audio_prefix": "xuejiang", "delivery_audio_prefix": "jisu", "lab_name": "激素检验窗口"},
        2: {"real_window": 3, "sample_name": "组织", "pickup_audio_prefix": "zuzhi", "delivery_audio_prefix": "mianyi", "lab_name": "免疫检验窗口"},
        1: {"real_window": 2, "sample_name": "唾液", "pickup_audio_prefix": "tuoye", "delivery_audio_prefix": "tiye", "lab_name": "体液检验窗口"},
        0: {"real_window": 1, "sample_name": "静脉血", "pickup_audio_prefix": "jingmaixue", "delivery_audio_prefix": "xuechanggui", "lab_name": "血常规检验窗口"},
    },

    "window_log_name": {
        "A": "a", "B": "b", "C": "c", "AB": "a、b", "AC": "a、c", "BC": "b、c", "ABC": "a、b、c",
    },
}


CARS = {
    1: {
        "car_id": 1,
        "peer_id": 2,
        "start_active": True,
        "use_peer_board1_result": False,
        "camera_url": "http://192.168.124.3:8080/stream?topic=/camera/rgb/image_raw",
        "home_pose": (0.0, 0.0, 7),
        "tcp": {
            "local_port": 9001,
            # 这里按 car2 的 camera_url 推断为 192.168.124.9；如果现场 car2 IP 不是它，请改这里。
            "peer_ip": "192.168.124.9",
            "peer_port": 9002,
        },
    },
    2: {
        "car_id": 2,
        "peer_id": 1,
        "start_active": False,
        "use_peer_board1_result": True,
        "camera_url": "http://192.168.124.9:8080/stream?topic=/camera/rgb/image_raw",
        "home_pose": (-0.069, -0.269, 7),
        "tcp": {
            "local_port": 9002,
            "peer_ip": "192.168.124.3",
            "peer_port": 9001,
        },
    },
}
