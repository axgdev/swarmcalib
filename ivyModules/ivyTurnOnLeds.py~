#!/usr/bin/env python

import time
#from thread import start_new_thread
from IvyCalibrationNode import *
from ivy.std_api import *

comm=IvyCalibrationNode()
comm.IvyInitStart()

#Just turn on leds
comm.IvySendKill(6)
time.sleep(2)
comm.IvyInitStop()
