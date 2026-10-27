#!/usr/bin/env python3
"""Local, single-car console. Python 3.6+, standard library only."""
import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import json
import os
from pathlib import Path
import re
import secrets
import signal
import subprocess
import threading
import time
from urllib.parse import parse_qs, urlsplit

ROOT = Path(__file__).resolve().parent
TOOLS = ROOT.parent
TOKEN = secrets.token_urlsafe(32)
LOCK = threading.RLock()
LOGS = deque(maxlen=1200)
DATA = {}
PREP = None
OBSERVER = None
BUSY = None
STARTED = None
MESSAGE = '请启动环境，初始化完成并确认定位后开始比赛。'
SEQUENCE = 0


def log(text):
    global SEQUENCE
    text = re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', text).rstrip()[:2000]
    with LOCK:
        SEQUENCE += 1
        LOGS.append({'id': SEQUENCE, 'time': time.time(), 'text': text})


def read_output(process, observer=False):
    for line in process.stdout:
        if observer and line.startswith('CONSOLE_JSON '):
            try:
                event = json.loads(line[13:])
                with LOCK:
                    DATA[event['kind']] = event
            except (ValueError, KeyError):
                log(line)
        else:
            log(line)
    process.stdout.close()
    log(('ROS 状态订阅' if observer else '环境脚本') + '已退出')


def spawn(command, observer=False):
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               universal_newlines=True, encoding='utf-8', errors='replace',
                               start_new_session=True, bufsize=1)
    threading.Thread(target=read_output, args=(process, observer), daemon=True).start()
    return process


def snapshot(after=0):
    with LOCK:
        ros = DATA.get('ros', {})
        fresh = time.time() - ros.get('time', 0) < 6
        running = PREP is not None and PREP.poll() is None
        return {'running': running, 'busy': BUSY, 'started': STARTED,
                'ready': fresh and ros.get('value', {}).get('ready', False),
                'ros_online': fresh, 'data': dict(DATA), 'message': MESSAGE,
                'log_cursor': SEQUENCE,
                'logs': [x for x in LOGS if x['id'] > after or after > SEQUENCE]}


def run_action(action):
    global PREP, OBSERVER, STARTED, MESSAGE, BUSY
    try:
        log('操作：' + action)
        if action == 'prepare':
            with LOCK:
                DATA.clear()
                STARTED = None
            if OBSERVER is not None and OBSERVER.poll() is None:
                os.killpg(OBSERVER.pid, signal.SIGTERM)
                OBSERVER.wait(timeout=5)
            PREP = spawn(['bash', str(TOOLS / 'prepare_race.sh')])
            OBSERVER = spawn(['bash', '-c',
                              'source ~/robot_ws/devel/setup.bash && exec "$1" -u "$2"',
                              'observer', ARGS.ros_python, str(ROOT / 'ros_observer.py')], True)
            MESSAGE = '环境启动中；等待启动服务就绪，并人工确认定位。'
        else:
            script = 'start_race.sh' if action == 'start' else 'stop_race.sh'
            process = subprocess.Popen(['bash', str(TOOLS / script)], stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, universal_newlines=True,
                                       encoding='utf-8', errors='replace', start_new_session=True)
            try:
                output, _ = process.communicate(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                output, _ = process.communicate(timeout=5)
                raise RuntimeError('命令超时，结果未知，请检查日志及车辆状态。' + output)
            log(output)
            if process.returncode:
                raise RuntimeError('脚本退出码：' + str(process.returncode))
            if action == 'start':
                if not re.search(r'success:\s*(?:True|true)\b', output):
                    raise RuntimeError('启动服务未接受请求：' + output)
                STARTED = time.time()
                MESSAGE = '启动请求已接受。'
            else:
                MESSAGE = '已发送停止请求，等待环境退出。'
                if PREP is not None:
                    try:
                        PREP.wait(timeout=25)
                        MESSAGE = '环境脚本已退出；ROS 状态仍由订阅反馈。'
                        STARTED = None
                    except subprocess.TimeoutExpired:
                        MESSAGE = '环境尚未退出，请查看日志；可再次点击全部停止。'
    except Exception as exc:
        MESSAGE = str(exc)
        log('ERROR ' + MESSAGE)
    finally:
        with LOCK:
            BUSY = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def respond(self, code, body, content_type='application/json; charset=utf-8'):
        body = body.encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Frame-Options', 'DENY')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == '/':
            html = (ROOT / 'index.html').read_text(encoding='utf-8')
            self.respond(200, html.replace('__TOKEN__', TOKEN), 'text/html; charset=utf-8')
        elif path == '/api/status':
            try:
                after = int(parse_qs(urlsplit(self.path).query).get('after', ['0'])[0])
            except ValueError:
                after = 0
            self.respond(200, json.dumps(snapshot(after), ensure_ascii=False))
        elif path == '/api/logs':
            with LOCK:
                text = '\n'.join(time.strftime('%H:%M:%S', time.localtime(x['time'])) + ' ' + x['text'] for x in LOGS)
            self.respond(200, text, 'text/plain; charset=utf-8')
        else:
            self.respond(404, '{}')

    def do_POST(self):
        global BUSY
        if self.headers.get('X-Console-Token') != TOKEN:
            return self.respond(403, '{"error":"Invalid token"}')
        action = self.path[5:]
        if self.path != '/api/' + action or action not in ('prepare', 'start', 'stop'):
            return self.respond(404, '{}')
        with LOCK:
            state = snapshot()
            if BUSY or (action == 'prepare' and (state['running'] or state['ready'])):
                return self.respond(409, '{"error":"操作进行中或环境已经启动"}')
            if action == 'start' and (not state['ready'] or STARTED is not None):
                return self.respond(409, '{"error":"服务未就绪或本次已放行"}')
            BUSY = action
            threading.Thread(target=run_action, args=(action,), daemon=True).start()
        self.respond(202, '{"accepted":true}')


class Server(ThreadingMixIn, HTTPServer):
    daemon_threads = True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8090)
    parser.add_argument('--ros-python', default='python', help='Python with rospy installed')
    ARGS = parser.parse_args()
    server = Server((ARGS.host, ARGS.port), Handler)
    print('Console: http://<robot-ip>:%d — trusted local network only' % ARGS.port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if OBSERVER is not None and OBSERVER.poll() is None:
            os.killpg(OBSERVER.pid, signal.SIGTERM)
        print('Console closed. Robot environment is unchanged; use stop_race.sh to stop it.')
