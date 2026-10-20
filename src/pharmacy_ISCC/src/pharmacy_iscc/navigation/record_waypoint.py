#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Record a stopped robot's map-frame pose. Never publishes motion or goals."""
from __future__ import print_function
import argparse
import datetime
import hashlib
import json
import math
from pharmacy_iscc.common.numeric import finite
import os
import shutil
import sys
import tempfile
import time

NAMES = ['home', 'board1', 'A', 'B', 'C', 'board2', 'lab1', 'lab2', 'lab3', 'lab4']


def map_identity(path):
    import yaml
    path = os.path.realpath(os.path.expanduser(path))
    raw = open(path, 'rb').read()
    meta = yaml.safe_load(raw)
    image = os.path.join(os.path.dirname(path), meta['image'])
    image_raw = open(image, 'rb').read()
    return {'yaml': path, 'sha256': hashlib.sha256(raw + b'\0' + image_raw).hexdigest(),
            'resolution': float(meta['resolution']), 'origin': list(meta['origin'])}

def checked_pose(translation, quaternion):
    if not finite(list(translation) + list(quaternion)):
        raise ValueError('Non-finite TF pose')
    norm = math.sqrt(sum(v*v for v in quaternion))
    if norm < 1e-8:
        raise ValueError('Invalid zero quaternion')
    x, y, z, w = [float(v)/norm for v in quaternion]
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return {'x': float(translation[0]), 'y': float(translation[1]),
            'yaw_rad': yaw, 'yaw_deg': math.degrees(yaw), 'quaternion': [x,y,z,w]}

def save_record(path, identity, car_id, name, pose, replace=False):
    path = os.path.realpath(os.path.expanduser(path))
    folder = os.path.dirname(path)
    if not os.path.isdir(folder):
        os.makedirs(folder)
    data = {'schema': 1, 'frame': 'map', 'base_frame': 'base_footprint',
            'car_id': car_id, 'map': identity, 'points': {}}
    if os.path.isfile(path):
        data = json.load(open(path))
        if data['map']['sha256'] != identity['sha256'] or data['car_id'] != car_id:
            raise ValueError('Different map or car: use a different --output file')
        if data.get('ros_map_sha256') != pose['ros_map_sha256']:
            raise ValueError('Loaded /map changed: reload the same saved map before recording')
        if name in data['points'] and not replace:
            raise ValueError('Point already exists; use --replace to deliberately re-record it')
        shutil.copy2(path, path + '.bak.' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    data['ros_map_sha256'] = pose['ros_map_sha256']
    data['points'][name] = pose
    fd, temporary = tempfile.mkstemp(prefix='.waypoint_', dir=folder)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(data, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.flush(); os.fsync(stream.fileno())
        os.rename(temporary, path)  # Atomic replacement on the vehicle's Linux filesystem.
    finally:
        if os.path.exists(temporary): os.unlink(temporary)
    return data

def capture(identity, max_age, timeout):
    import array
    import rospy
    import tf
    from nav_msgs.msg import OccupancyGrid, Odometry
    rospy.init_node('record_pharmacy_waypoint', anonymous=True, disable_signals=True)
    grid = rospy.wait_for_message('/map', OccupancyGrid, timeout=timeout)
    if grid.header.frame_id.lstrip('/') != 'map':
        raise ValueError('/map must use the map frame')
    if abs(grid.info.resolution-identity['resolution']) > 1e-6:
        raise ValueError('Loaded /map resolution differs from --map-file')
    origin = checked_pose([grid.info.origin.position.x, grid.info.origin.position.y, 0],
                          [grid.info.origin.orientation.x, grid.info.origin.orientation.y,
                           grid.info.origin.orientation.z, grid.info.origin.orientation.w])
    if max(abs(origin[k]-v) for k,v in zip(['x','y','yaw_rad'],identity['origin'])) > 1e-5:
        raise ValueError('Loaded /map origin differs from --map-file')
    cells = array.array('b', grid.data)
    cells_raw = cells.tobytes() if hasattr(cells, 'tobytes') else cells.tostring()
    grid_hash = hashlib.sha256(cells_raw).hexdigest()
    motion = []
    def odom(msg):
        age = (rospy.Time.now()-msg.header.stamp).to_sec()
        if msg.header.stamp.to_sec() <= 0 or age > max_age or age < -1:
            return
        t = msg.twist.twist
        motion.append((time.time(), math.hypot(t.linear.x,t.linear.y), abs(t.angular.z)))
        del motion[:-2]
    sub = rospy.Subscriber('/odometry/filtered', Odometry, odom, queue_size=2)
    listener = tf.TransformListener()
    deadline = time.time() + timeout
    error = 'No map -> base_footprint TF or fresh stationary odometry'
    while time.time() < deadline and not rospy.is_shutdown():
        try:
            recent = list(motion)
            if len(recent) < 2 or any(not finite(r) or time.time()-r[0]>max_age or r[1]>.02 or r[2]>.05 for r in recent):
                raise ValueError('Stop the robot: require fresh odometry below 0.02 m/s and 0.05 rad/s')
            stamp = listener.getLatestCommonTime('map','base_footprint')
            age = (rospy.Time.now()-stamp).to_sec()
            if stamp.to_sec() <= 0 or age > max_age or age < -1:
                raise ValueError('Map TF is stale or has an invalid time')
            translation, quaternion = listener.lookupTransform('map','base_footprint',stamp)
            pose = checked_pose(translation, quaternion)
            pose.update(tf_stamp=stamp.to_sec(), tf_age_sec=age, ros_map_sha256=grid_hash,
                        recorded_at=datetime.datetime.now().isoformat())
            return pose
        except (tf.Exception, ValueError) as exc:
            error = str(exc)
        time.sleep(.1)
    raise ValueError(error)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name', nargs='?', choices=NAMES)
    parser.add_argument('--map-file')
    parser.add_argument('--output', default='~/waypoints/pharmacy_points.json')
    parser.add_argument('--car-id', type=int, choices=[1,2], default=1)
    parser.add_argument('--replace', action='store_true')
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--timeout', type=float, default=10.)
    parser.add_argument('--max-age', type=float, default=2.)
    args = parser.parse_args()
    try:
        if args.list:
            data = json.load(open(os.path.expanduser(args.output)))
        else:
            if not args.name or not args.map_file: parser.error('name and --map-file are required')
            if args.timeout <= 0 or args.max_age <= 0: parser.error('timeouts must be positive')
            identity = map_identity(args.map_file)
            data = save_record(args.output,identity,args.car_id,args.name,
                               capture(identity,args.max_age,args.timeout),args.replace)
            print('SAVED %s: x=%.4f y=%.4f yaw=%.4f rad (%.1f deg)' %
                  ((args.name,)+tuple(data['points'][args.name][k] for k in ['x','y','yaw_rad','yaw_deg'])))
        print('Recorded: '+', '.join(n for n in NAMES if n in data['points']))
        print('Missing: '+', '.join(n for n in NAMES if n not in data['points']))
        print('File: '+os.path.expanduser(args.output))
        return 0
    except Exception as exc:
        print('NOT SAVED: '+str(exc),file=sys.stderr)
        return 1

