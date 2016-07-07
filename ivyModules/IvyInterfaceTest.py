#!/usr/bin/env python

import time
from thread import start_new_thread
from IvyCalibrationNode import *
from std_api import *

comm=IvyCalibrationNode()
comm.IvyInitStart()

#dostuff
comm.IvySendUnKill(5)
comm.IvySendCalib(5,58,0.5)
time.sleep(1)
#domorestuff
comm.IvySendCalib(5,59,0.7)
time.sleep(1)
comm.IvySendCalib(5,60,0.9)
comm.IvyGetPos()
comm.IvySendKill(5)
comm.IvyInitStop()
