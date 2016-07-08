#!/usr/bin/env python

import time
#from thread import start_new_thread
from IvyCalibrationNode import *
from ivy.std_api import *

comm=IvyCalibrationNode()
comm.IvyInitStart()

comm1=IvyCalibrationNode()
comm1.IvyInitStart()

time.sleep(4)

comm.IvyInitStop()
comm1.IvyInitStop()
