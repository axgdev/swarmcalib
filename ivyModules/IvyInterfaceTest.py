#!/usr/bin/env python

import time
from thread import start_new_thread
from IvyCalibrationNode import *
from std_api import *

comm=IvyCalibrationNode()
comm.IvyInitStart()

#dostuff
comm.IvySendUnKill(6)
#comm.IvySendCalib(5,58,0.5)
time.sleep(1)
#domorestuff
#comm.IvySendCalib(5,59,0.7)
#time.sleep(1)

#comm.IvyGetPos()
comm.IvySwitchBlock(6, 2)
time.sleep(1)
comm.IvySwitchBlock(6, 4)
time.sleep(0.5)
comm.IvySendKill(6)
comm.IvyInitStop()
