#!/usr/bin/env python
from __future__ import print_function
import glob
import imp
import json
import math
import os
import shutil
import tempfile
import unittest

ROOT=os.path.dirname(os.path.abspath(__file__))
SCRIPT=os.path.join(ROOT,'record_waypoint.py')
if not os.path.isfile(SCRIPT):
    SCRIPT=os.path.join(os.path.dirname(ROOT),'scripts','record_waypoint.py')
m=imp.load_source('waypoint_recorder_under_test',SCRIPT)

class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.mkdtemp(prefix='waypoint_test_')
        self.path=self.folder+'/points.json'
        self.identity={'sha256':'map-one','yaml':'test.yaml'}
        self.pose=dict(x=1.,y=2.,yaw_rad=.5,ros_map_sha256='grid-one')
    def tearDown(self):shutil.rmtree(self.folder)
    def save(self,name='A',**kwargs):return m.save_record(self.path,self.identity,1,name,self.pose,**kwargs)
    def test_orientation_and_invalid_tf(self):
        p=m.checked_pose([1,2,0],[0,0,math.sqrt(.5),math.sqrt(.5)])
        self.assertAlmostEqual(p['yaw_rad'],math.pi/2)
        for q in [[0,0,0,0],[0,0,float('nan'),1]]:
            with self.assertRaises(ValueError):m.checked_pose([1,2,0],q)
    def test_duplicate_refuses_without_changing_original(self):
        self.save();before=open(self.path,'rb').read()
        with self.assertRaises(ValueError):self.save()
        self.assertEqual(open(self.path,'rb').read(),before)
    def test_replace_preserves_backup_and_other_points(self):
        self.save();self.save('B');self.pose['x']=3.;self.save(replace=True)
        data=json.load(open(self.path));self.assertEqual(data['points']['A']['x'],3.)
        self.assertEqual(data['points']['B']['x'],1.)
        backups=glob.glob(self.path+'.bak.*');self.assertEqual(len(backups),2)
        self.assertTrue(any(json.load(open(p))['points']['A']['x']==1. for p in backups))
    def test_map_car_and_loaded_grid_mismatch_are_rejected(self):
        self.save();before=open(self.path,'rb').read()
        for identity,car,pose in [({'sha256':'different'},1,self.pose),(self.identity,2,self.pose),
                                  (self.identity,1,dict(self.pose,ros_map_sha256='different'))]:
            with self.assertRaises(ValueError):m.save_record(self.path,identity,car,'B',pose)
        self.assertEqual(open(self.path,'rb').read(),before)
    def test_map_image_change_alters_identity(self):
        image=self.folder+'/map.pgm';config=self.folder+'/map.yaml'
        open(image,'wb').write(b'P5\n1 1\n255\n\xff')
        open(config,'w').write('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
        first=m.map_identity(config)['sha256']
        open(image,'wb').write(b'P5\n1 1\n255\n\x00')
        self.assertNotEqual(first,m.map_identity(config)['sha256'])

if __name__=='__main__':unittest.main(verbosity=2)
