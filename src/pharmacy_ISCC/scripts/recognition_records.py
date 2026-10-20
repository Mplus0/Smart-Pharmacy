#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""本机识别记录：现有视觉/任务进程共享 SQLite，任务结束后导出 Excel。"""
import hashlib
import json
import os
import sqlite3
import threading
from contextlib import closing

import rospy


from pharmacy_iscc.config import records_path


LOG_DIR = records_path()


def database_path(car_id):
    return os.path.join(LOG_DIR, 'car%d.sqlite3' % int(car_id))


def record_result(car_id, board, content_type, parsed_content):
    """只记录调用点已经确认的结果；不执行识别或任务选择。"""
    try:
        # JSON 保留解析字段含义，并转义控制字符，Excel 中显示的文本即 MD5 输入。
        content = json.dumps(parsed_content, ensure_ascii=False,
                             sort_keys=True, separators=(',', ':'))
        digest = hashlib.md5(content.encode('utf-8')).hexdigest()
        path = database_path(car_id)
        if not os.path.isfile(path):
            rospy.logwarn('MD5 record skipped: no active task database')
            return
        with closing(sqlite3.connect(path, timeout=5.0)) as conn:
            with conn:
                # INSERT ... SELECT 与结束事件互斥，避免导出后又插入迟到记录。
                conn.execute(
                    'INSERT INTO records (board, kind, content, md5) '
                    'SELECT ?, ?, ?, ? FROM active_run WHERE slot=1',
                    (board, content_type, content, digest))
    except Exception as exc:
        # 记录故障不改变原有识别、导航和调度分支。
        rospy.logerr('MD5 record failed: %s', exc)


class RecognitionRecords(object):
    def __init__(self, car_id):
        # 提前检查依赖，避免完成整场后才发现无法生成 Excel。
        from openpyxl import Workbook
        self.workbook_class = Workbook
        self.car_id = car_id
        self.exported_path = None
        self.export_lock = threading.Lock()
        if not os.path.isdir(LOG_DIR):
            try:
                os.makedirs(LOG_DIR)
            except OSError:
                if not os.path.isdir(LOG_DIR):
                    raise
        with closing(sqlite3.connect(database_path(car_id), timeout=5.0)) as conn:
            with conn:
                conn.execute('CREATE TABLE IF NOT EXISTS active_run '
                             '(slot INTEGER PRIMARY KEY)')
                conn.execute('CREATE TABLE IF NOT EXISTS records '
                             '(id INTEGER PRIMARY KEY, '
                             'board TEXT, kind TEXT, content TEXT, md5 TEXT)')
                # 每次任务启动重新记录，只保留本次运行的数据。
                conn.execute('DELETE FROM records')
                conn.execute('INSERT OR REPLACE INTO active_run VALUES (1)')

    def record(self, board, content_type, parsed_content):
        record_result(self.car_id, board, content_type, parsed_content)

    def export(self):
        """关闭本次记录并导出；正常结束与 shutdown 重入时不重复导出。"""
        with self.export_lock:
            return self._export()

    def _export(self):
        if self.exported_path is not None:
            return self.exported_path
        with closing(sqlite3.connect(database_path(self.car_id), timeout=5.0)) as conn:
            with conn:
                conn.execute('DELETE FROM active_run')
                rows = conn.execute(
                    'SELECT board, kind, content, md5 FROM records '
                    'ORDER BY id').fetchall()
        workbook = self.workbook_class()
        sheet = workbook.active
        sheet.title = u'识别记录'
        sheet.append([u'序号', u'识别板', u'内容类型', u'具体内容', u'MD5'])
        for index, row in enumerate(rows, 1):
            sheet.append([index] + list(row))
        sheet.freeze_panes = 'A2'
        sheet.auto_filter.ref = sheet.dimensions
        for column, width in [('A', 10), ('B', 14), ('C', 18), ('D', 80), ('E', 36)]:
            sheet.column_dimensions[column].width = width
        # 固定结果文件，每次运行结束直接覆盖上次结果。
        path = os.path.join(LOG_DIR, 'recognition.xlsx')
        workbook.save(path)
        self.exported_path = path
        return path
