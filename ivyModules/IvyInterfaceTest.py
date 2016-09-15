#!/usr/bin/env python

import time
#from thread import start_new_thread
from IvyCalibrationNode import *
from ivy.std_api import *

comm=IvyCalibrationNode()
comm.IvyInitStart()



#dostuff
comm.IvySendUnKill(5)

comm.IvySendCalib(5, 58, 0.00640)
comm.IvySendCalib(5, 59, 0.0065)
"""
#domorestuff
#comm.IvySendCalib(5,59,0.7)
#time.sleep(1)

#comm.IvyGetPos()
comm.IvySendSwitchBlock(6, 2)
time.sleep(2)
comm.IvySendSwitchBlock(6, 4)
time.sleep(1)
"""
time.sleep(2)
comm.IvySendKill(6)
comm.IvyInitStop()
