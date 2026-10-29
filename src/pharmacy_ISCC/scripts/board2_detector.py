# -*- coding: utf-8 -*-
"""Board-two local template recognition, using frames supplied by the ROS node."""
import os
import atexit
import base64
import errno
import fcntl
import json
import select
import subprocess
import time
import cv2
import rospy
from pharmacy_iscc.config import COMMON, package_path, records_path

BOARD2_CFG = COMMON["board2"]
BOARD2_TEMPLATE_DIR = COMMON["paths"]["board2_template_dir"]
BOARD2_MATCH_THRESHOLD = BOARD2_CFG["match_threshold"]
BOARD2_SCORE_MARGIN = BOARD2_CFG["score_margin"]
BOARD2_ROI = BOARD2_CFG["roi"]
BOARD2_TEMPLATE_SCALES = BOARD2_CFG["template_scales"]


def preprocess_board2_img(img):
    """
    识别板二模板匹配预处理：
    1. 灰度化；
    2. 高斯滤波；
    3. Otsu 二值化。
    """
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()

    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return binary


def load_board2_templates():
    """
    加载识别板二模板。

    模板命名：
    - free.png
    - busy_5.png ~ busy_10.png
    """
    templates = {}

    template_files = {
        "free": "free.png",
        "busy_5": "busy_5.png",
        "busy_6": "busy_6.png",
        "busy_7": "busy_7.png",
        "busy_8": "busy_8.png",
        "busy_9": "busy_9.png",
        "busy_10": "busy_10.png"
    }

    for label, filename in template_files.items():
        path = os.path.join(BOARD2_TEMPLATE_DIR, filename)
        img = cv2.imread(path)

        if img is None:
            rospy.logwarn("识别板二模板未找到: %s", path)
            continue

        templates[label] = preprocess_board2_img(img)

    rospy.logwarn("识别板二模板加载完成，数量: %d", len(templates))
    return templates


def crop_board2_roi(frame):
    """裁剪识别板二 ROI，当前默认全图。"""
    h, w = frame.shape[:2]
    x1_rate, y1_rate, x2_rate, y2_rate = BOARD2_ROI

    x1 = int(w * x1_rate)
    y1 = int(h * y1_rate)
    x2 = int(w * x2_rate)
    y2 = int(h * y2_rate)

    return frame[y1:y2, x1:x2]


def match_one_template(search_img, template_img):
    """单模板多尺度匹配，返回最高匹配分数。"""
    best_score = -1.0
    sh, sw = search_img.shape[:2]

    for scale in BOARD2_TEMPLATE_SCALES:
        tw = int(template_img.shape[1] * scale)
        th = int(template_img.shape[0] * scale)

        if tw <= 10 or th <= 10:
            continue
        if tw >= sw or th >= sh:
            continue

        resized_template = cv2.resize(template_img, (tw, th))
        result = cv2.matchTemplate(search_img, resized_template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, _ = cv2.minMaxLoc(result)

        if max_val > best_score:
            best_score = max_val

    return best_score


def detect_board2_status(frame, board2_templates, visit=None, is_active=None):
    """检测识别板二状态，返回 label 和 score。"""

    if BOARD2_CFG.get('result_source') == 'yolo':
        return detect_board2_yolo(frame, visit, is_active)

    if len(board2_templates) == 0:
        return None, 0.0

    roi = crop_board2_roi(frame)
    search_img = preprocess_board2_img(roi)

    scores = []
    for label, template_img in board2_templates.items():
        score = match_one_template(search_img, template_img)
        scores.append((label, score))

    if len(scores) == 0:
        return None, 0.0

    scores.sort(key=lambda x: x[1], reverse=True)
    best_label, best_score = scores[0]

    if len(scores) >= 2:
        second_label, second_score = scores[1]
    else:
        second_label, second_score = None, -1.0

    if (best_score >= BOARD2_MATCH_THRESHOLD and
            (best_score - second_score) >= BOARD2_SCORE_MARGIN):
        return best_label, best_score

    rospy.loginfo_throttle(
        1.0,
        "识别板二匹配不稳定: best=%s %.3f second=%s %.3f",
        best_label, best_score, second_label, second_score
    )
    return None, best_score


_yolo_worker = None
_yolo_stderr = None


def close_board2_yolo():
    global _yolo_worker, _yolo_stderr
    if _yolo_worker is not None:
        if _yolo_worker.poll() is None:
            _yolo_worker.terminate()
            deadline = time.time() + 1.0
            while _yolo_worker.poll() is None and time.time() < deadline:
                time.sleep(0.02)
            if _yolo_worker.poll() is None:
                _yolo_worker.kill()
        _yolo_worker.wait()
        for stream in (_yolo_worker.stdin, _yolo_worker.stdout):
            if stream is not None:
                stream.close()
        _yolo_worker = None
    if _yolo_stderr is not None:
        _yolo_stderr.close()
        _yolo_stderr = None


def _read_worker_result(timeout, is_active=None):
    deadline = time.time() + timeout
    data = b''
    while time.time() < deadline and not rospy.is_shutdown():
        if is_active is not None and not is_active():
            raise RuntimeError('YOLO visit cancelled')
        readable, _, _ = select.select([_yolo_worker.stdout], [], [], 0.1)
        if readable:
            chunk = os.read(_yolo_worker.stdout.fileno(), 65536)
            if not chunk:
                raise RuntimeError('YOLO worker exited')
            data += chunk
            if len(data) > 65536:
                raise RuntimeError('YOLO worker response too long')
            if b'\n' in data:
                return json.loads(data.split(b'\n', 1)[0].decode('utf-8'))
    raise RuntimeError('YOLO worker timed out or cancelled')


def _write_worker_request(payload, timeout, is_active=None):
    descriptor = _yolo_worker.stdin.fileno()
    flags = fcntl.fcntl(descriptor, fcntl.F_GETFL)
    fcntl.fcntl(descriptor, fcntl.F_SETFL, flags | os.O_NONBLOCK)
    deadline = time.time() + timeout
    offset = 0
    while offset < len(payload) and time.time() < deadline and not rospy.is_shutdown():
        if is_active is not None and not is_active():
            raise RuntimeError('YOLO visit cancelled')
        _, writable, _ = select.select([], [_yolo_worker.stdin], [], 0.1)
        if writable:
            try:
                offset += os.write(descriptor, payload[offset:offset + 65536])
            except OSError as exc:
                if exc.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                    raise
    if offset != len(payload):
        raise RuntimeError('YOLO send timed out or cancelled')


def detect_board2_yolo(frame, visit, is_active=None):
    """Use the node-owned frame; worker owns no camera, ROS topics or TCP port."""
    global _yolo_worker, _yolo_stderr
    try:
        timeout = float(BOARD2_CFG.get('yolo_timeout_sec', 10.0))
        if _yolo_worker is None or _yolo_worker.poll() is not None:
            close_board2_yolo()
            evidence = os.path.join(records_path(), 'board2_yolo')
            if not os.path.isdir(evidence):
                os.makedirs(evidence)
            _yolo_stderr = open(os.path.join(evidence, 'worker_stderr.log'), 'ab')
            environment = os.environ.copy()
            environment.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='2', PYTHONDONTWRITEBYTECODE='1')
            _yolo_worker = subprocess.Popen([
                os.path.expanduser(BOARD2_CFG['yolo_python']),
                os.path.join(os.path.dirname(__file__), 'board2_yolo_worker.py'),
                '--model-dir', package_path(BOARD2_CFG['yolo_model_dir']),
                '--evidence-dir', evidence], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=_yolo_stderr, env=environment, bufsize=0)
            if not _read_worker_result(timeout, is_active).get('ready'):
                raise RuntimeError('YOLO worker did not become ready')
        ok, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        if not ok:
            raise ValueError('Camera frame JPEG encoding failed')
        request = {'image': base64.b64encode(encoded.tostring()).decode('ascii'), 'visit': visit}
        deadline = time.time() + timeout
        _write_worker_request((json.dumps(request) + '\n').encode('ascii'), timeout, is_active)
        result = _read_worker_result(max(0.0, deadline - time.time()), is_active)
        label = result.get('label')
        if label not in ('free', 'busy_5', 'busy_6', 'busy_7', 'busy_8', 'busy_9', 'busy_10'):
            rospy.loginfo_throttle(2.0, 'Board2 YOLO has no stable result: %s', result.get('reason'))
            return None, 0.0
        # Existing Python2 Chinese log formats expect the original byte-string labels.
        label = str(label)
        score = float(result['score'])
        rospy.loginfo('Board2 YOLO visit=%s label=%s confidence=%.6f', visit, label, score)
        return label, score
    except Exception as exc:
        rospy.logwarn_throttle(2.0, 'Board2 YOLO unavailable: %s', exc)
        close_board2_yolo()
        return None, 0.0


atexit.register(close_board2_yolo)


