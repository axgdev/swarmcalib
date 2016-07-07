#!/usr/bin/env python

import time
from thread import start_new_thread
from Ivy_Calibration_Node import *
from ivy.std_api import *

IvyInitStart()
pose=IvyGetPos()
print(pose)
IvySendParams(5, pose.x, pose.y, pose.theta)
time.sleep(2)

#domorestuff
pose = IvyGetPos()
print(pose)
IvySendParams(5, pose.x, pose.y, pose.theta)
IvyStop()
