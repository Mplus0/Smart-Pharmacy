# -*- coding: utf-8 -*-
import math
import cv2


def distance(point1, point2):
    """计算两个点之间的欧氏距离。"""
    return math.sqrt((point1[0] - point2[0]) ** 2 +
                     (point1[1] - point2[1]) ** 2)


def find_contours(binary_img, mode, method):
    """兼容不同 OpenCV 版本的 findContours 返回值。"""
    result = cv2.findContours(binary_img, mode, method)
    if len(result) == 3:
        _, contours, _ = result
    else:
        contours, _ = result
    return contours


def rotate_frame(frame, angle):
    """按原代码角度旋转图像。"""
    height, width = frame.shape[:2]
    center = (width // 2, height // 2)
    matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    return cv2.warpAffine(frame, matrix, (width, height))
