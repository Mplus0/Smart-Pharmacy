from __future__ import print_function
import glob
import json
import math
import os
import shutil
import sys
import tempfile
import unittest

from pharmacy_iscc.navigation import waypoint_console as m


class Object(object):
    def __init__(self, **kwargs): self.__dict__.update(kwargs)


def grid(data=None, yaw=0.):
    origin = Object(position=Object(x=0.,y=0.,z=0.),
                    orientation=Object(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2)))
    return Object(header=Object(frame_id='map'), info=Object(width=2,height=2,resolution=1.,origin=origin),
                  data=[0,100,-1,0] if data is None else data)


class Client(object):
    def __init__(self, state=3, interrupted=False, acknowledge=True):
        self.state,self.interrupted,self.acknowledge=state,interrupted,acknowledge
        self.sent=[]; self.cancelled=0
    def send_goal(self, goal): self.sent.append(goal)
    def cancel_goal(self): self.cancelled+=1
    def wait_for_result(self, duration):
        if self.interrupted:
            self.interrupted=False; raise KeyboardInterrupt()
        return self.acknowledge if self.cancelled else self.state is not None
    def get_state(self): return self.state


class Clock(object):
    def __init__(self): self.value=0.
    def time(self): self.value+=.1; return self.value


class Tests(unittest.TestCase):
    def test_heading_is_degrees_and_nonfinite_is_rejected(self):
        self.assertAlmostEqual(m.target_pose(1,2,90)['yaw_rad'],math.pi/2)
        self.assertAlmostEqual(m.target_pose(1,2,450)['yaw_deg'],90)
        for values in [('nan',0,0),(0,'inf',0),(0,0,'-inf')]:
            with self.assertRaises(ValueError): m.target_pose(*values)

    def test_unknown_occupied_outside_and_rotated_map(self):
        m.validate_target(grid(),m.target_pose(.5,.5,0))
        for x,y in [(1.5,.5),(.5,1.5),(-.01,0),(2.,0)]:
            with self.assertRaises(ValueError): m.validate_target(grid(),m.target_pose(x,y,0))
        m.validate_target(grid(yaw=math.pi/2),m.target_pose(-.5,.5,0))

    def test_map_identity_binds_cells_and_origin(self):
        a=m.grid_identity(grid())['sha256']
        self.assertNotEqual(a,m.grid_identity(grid([0,0,-1,0]))['sha256'])
        self.assertNotEqual(a,m.grid_identity(grid(yaw=math.pi/2))['sha256'])

    def test_arrival_rejects_position_heading_and_wraps_angles(self):
        target=m.target_pose(0,0,180)
        accepted=m.arrival_check(m.target_pose(.03,0,-179),target,.05,3.)
        self.assertAlmostEqual(accepted['yaw_error_deg'],1.)
        for pose in [m.target_pose(.051,0,180),m.target_pose(0,0,176),m.target_pose(0,0,140)]:
            with self.assertRaises(ValueError):m.arrival_check(pose,target,.05,3.)

    def test_settling_restarts_after_leaving_tolerance(self):
        calls=[0];clock=Clock();target=m.target_pose(0,0,0)
        def sample():
            calls[0]+=1
            return m.target_pose(.1 if calls[0]==2 else 0,0,0)
        actual,check=m.settled_pose(sample,target,.05,3.,.4,5.,clock.time)
        self.assertGreaterEqual(calls[0],5)
        self.assertGreaterEqual(check['stable_sec'],.4)

    def test_action_success_with_bad_pose_is_not_accepted(self):
        clock=Clock()
        with self.assertRaises(ValueError):
            m.settled_pose(lambda:m.target_pose(0,0,40),m.target_pose(0,0,0),.05,3.,.2,1.,clock.time)

    def test_moving_or_stale_samples_cannot_pass_settling(self):
        clock=Clock()
        def sample():raise ValueError('Robot still moving or localization stale')
        with self.assertRaises(ValueError):m.settled_pose(sample,None,.05,3.,.2,1.,clock.time)

    def test_record_isolated_by_map_vehicle_duplicate_and_backup(self):
        folder=tempfile.mkdtemp()
        try:
            path=os.path.join(folder,'points.json'); identity=m.grid_identity(grid())
            pose=m.target_pose(.5,.5,90)
            m.save_point(path,'car20',identity,'A',pose,pose)
            with open(path,'rb') as stream: before=stream.read()
            for vehicle,meta in [('car6',identity),('car20',dict(identity,sha256='other'))]:
                with self.assertRaises(ValueError): m.save_point(path,vehicle,meta,'B',pose,pose)
            with self.assertRaises(ValueError): m.save_point(path,'car20',identity,'A',pose,pose)
            with open(path,'rb') as stream: self.assertEqual(before,stream.read())
            m.save_point(path,'car20',identity,'A',m.target_pose(.6,.6,0),pose,True)
            self.assertEqual(len(glob.glob(path+'.bak.*')),1)
            with open(path) as stream: saved=json.load(stream)
            self.assertEqual(saved['points']['A']['requested']['x'],.5)
            self.assertEqual(saved['points']['A']['pose']['x'],.6)
        finally: shutil.rmtree(folder)

    def test_success_sends_one_goal(self):
        c=Client(); self.assertTrue(m.run_goal(c,'goal',1.,float,lambda:True))
        self.assertEqual(c.sent,['goal']);self.assertEqual(c.cancelled,0)

    def test_abort_never_reports_arrival(self):
        c=Client(state=4)
        with self.assertRaises(ValueError): m.run_goal(c,'goal',1.,float,lambda:True)
        self.assertEqual(c.cancelled,1)

    def test_interrupt_cancels_only_own_goal(self):
        c=Client(interrupted=True)
        with self.assertRaises(KeyboardInterrupt): m.run_goal(c,'goal',1.,float,lambda:True)
        self.assertEqual(c.cancelled,1);self.assertEqual(c.sent,['goal'])

    def test_timeout_cancels_goal(self):
        c=Client(state=None); original=m.time;m.time=Clock()
        try:
            with self.assertRaises(ValueError): m.run_goal(c,'goal',.5,float,lambda:True)
            self.assertEqual(c.cancelled,1)
        finally: m.time=original

    def test_map_change_cancels_goal(self):
        c=Client(); checks=iter([True,False])
        with self.assertRaises(ValueError): m.run_goal(c,'goal',1.,float,lambda:next(checks))
        self.assertEqual(c.cancelled,1)

    def test_map_changed_before_send_never_sends_goal(self):
        c=Client()
        with self.assertRaises(ValueError): m.run_goal(c,'goal',1.,float,lambda:False)
        self.assertEqual(c.sent,[]);self.assertEqual(c.cancelled,0)

    def test_unacknowledged_cancel_stops_console(self):
        c=Client(interrupted=True,acknowledge=False)
        with self.assertRaises(RuntimeError): m.run_goal(c,'goal',1.,float,lambda:True)


if __name__=='__main__': unittest.main(verbosity=2)
