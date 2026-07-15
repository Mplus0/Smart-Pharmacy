#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import time
import cv2

CAMERA_URL = "http://192.168.124.3:8080/stream?topic=/camera/rgb/image_raw"
FRAME_ROTATE_ANGLE = 0
SAVE_ROOT = "/home/EPRobot/robot_ws/src/dataset"

VALID_LABELS = [
    "free",
    "busy_5",
    "busy_6",
    "busy_7",
    "busy_8",
    "busy_9",
    "busy_10"
]


def rotate_frame(frame, angle):
    h, w = frame.shape[:2]
    center = (w // 2, h // 2)
    matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    return cv2.warpAffine(frame, matrix, (w, h))


def main():
    if len(sys.argv) < 2:
        print("用法: python capture_board2_dataset.py free/busy_5/.../busy_10")
        return

    label = sys.argv[1]
    if label not in VALID_LABELS:
        print("非法类别:", label)
        return

    save_dir = os.path.join(SAVE_ROOT, label)
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    cap = cv2.VideoCapture(CAMERA_URL)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("摄像头打开失败")
        return

    print("当前采集类别:", label)
    print("按 s 保存当前帧，按 q 退出")

    count = len([x for x in os.listdir(save_dir) if x.lower().endswith(".jpg")])

    while True:
        for _ in range(5):
            cap.grab()

        ret, frame = cap.read()
        if not ret:
            print("读取失败")
            time.sleep(0.1)
            continue

        frame = rotate_frame(frame, FRAME_ROTATE_ANGLE)

        show = frame.copy()
        cv2.putText(
            show,
            "label: %s count: %d" % (label, count),
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2
        )

        cv2.imshow("capture_board2_dataset", show)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("s"):
            filename = "%04d.jpg" % count
            path = os.path.join(save_dir, filename)
            cv2.imwrite(path, frame)
            print("保存:", path)
            count += 1

        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
