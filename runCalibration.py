#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  calibration.py
#

import time
#To save the calibration parameters
import calibrationOutput
import calibrationV2

""" Logging imports """
import logging
import os

#Ivy cal node
import ivyModules.IvyCalibrationNode
import math

""" Superior logging capabilities. Way better than prints :P """

#Set the logger object with the name we have chosen
logger = logging.getLogger('cal')
#Set the logger level, for all cases. This will be configured for each handler
logger.setLevel(logging.DEBUG)

#Create the log folder if it does not exist
calibrationOutput.make_sure_path_exists("logs")

# create global file handler which logs debug messages
globalFileHandler = logging.FileHandler("logs/global.log")
globalFileHandler.setLevel(logging.DEBUG)
# create a file handler per session with timestamp of creation
sessionFileHandler = logging.FileHandler("logs/"+calibrationOutput.getFormattedTimeStamp()+".log")
sessionFileHandler.setLevel(logging.DEBUG)
# Output to console, we can choose to log only certain info, for now log all
consoleHandler = logging.StreamHandler()
consoleHandler.setLevel(logging.DEBUG)
# create formatter and add it to the handlers
#Use this formatter for showing the name of the calibration script
#formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
""" Formatter without name and level on each stamp (cleaner, shorter log)"""
formatter = logging.Formatter("%(asctime)s - %(message)s")
consoleHandler.setFormatter(formatter)
globalFileHandler.setFormatter(formatter)
sessionFileHandler.setFormatter(formatter)
# add the handlers to logger
logger.addHandler(consoleHandler)
logger.addHandler(globalFileHandler)
logger.addHandler(sessionFileHandler)


"""Initialization for main function"""
myCalibrator = calibrationV2.Calibrator(logger)
myCalibrator.setDeadZone(300,1250,250,950) #minX, maxX, minY, maxY
myCalibrator.setBasePosition(750,600)
myCalibrator.setPollingTime(0.005) #optimum: 0.005
myCalibrator.setAircraftID(5)


""" Reading previous best calibration parameters, uncomment if you want to use it """
"""
inputParams = calibrationOutput.loadCalibration()
if (len(inputParams) > 1)
    myCalibrator.bestPitch = inputParams[0]*math.pi/180
    myCalibrator.bestRoll = inputParams[1]*math.pi/180
    myCalibrator.absDiff = inputParams[2]*math.pi/180
"""

""" Set initial messages in the debug log """
logger.debug("*********NEW SESSION*********")
logger.debug("Deadzone-> minX=%f, maxX=%f, minY=%f, maxY=%f" %
                        (myCalibrator.minX, myCalibrator.maxX,
                         myCalibrator.minY, myCalibrator.maxY))
logger.debug("Base position: baseX=%f, baseY=%f" %
                            (myCalibrator.baseX, myCalibrator.baseX))
logger.debug("PollingTime = %f" % myCalibrator.pollingTime)
logger.debug("XPID = %f, %f, %f / YPID = %f, %f, %f" %
            (myCalibrator.targetXController.p, myCalibrator.targetXController.i,
             myCalibrator.targetXController.d, myCalibrator.targetYController.p,
             myCalibrator.targetYController.i, myCalibrator.targetYController.d))

""" End of debug log initial messages """

myCalibrator.myIvyCalNode.IvyInitStart()
myCalibrator.sendParametersToCopter(0, 0, 0) #We make sure pitch, roll and yaw are 0 at start
myCalibrator.myIvyCalNode.IvySendCalib(myCalibrator.aircraftID, 58, 0)
myCalibrator.myIvyCalNode.IvySendCalib(myCalibrator.aircraftID, 59, 0)
myCalibrator.unkillCopter()
time.sleep(3) #For the camera to detect the initial position
myCalibrator.sendStartMode() #I uncommented this for simulation purposes
time.sleep(1.75) #When the copter turns on, there are no lights until a few seconds

i = 0;
while(myCalibrator.calibIter <= 100):
    myCalibrator.getXYCoordinates()
    if (myCalibrator.isInDeadZone()):
        myCalibrator.myIvyCalNode.SetInDeadZone(True)
        myCalibrator.killCopter()
        myCalibrator.sendParametersToCopter(0, 0, 0)
        #myCalibrator.myIvyCalNode.IvySendCalib(myCalibrator.aircraftID, 58, -myCalibrator.bestRoll)
        #myCalibrator.myIvyCalNode.IvySendCalib(myCalibrator.aircraftID, 59, myCalibrator.bestPitch)

        myCalibrator.myIvyCalNode.IvyInitStop()
        break;
    #time.sleep(3)
    myCalibrator.followTarget()
    i=i+1
    time.sleep(myCalibrator.pollingTime)


myCalibrator.sendParametersToCopter(0, 0, 0)
logger.debug("final calib values: Roll: " +str(-myCalibrator.rollCalib) + "  Pitch: " + str(myCalibrator.pitchCalib) + " Calib iter: " + str(myCalibrator.calibIter))
time.sleep(1)
myCalibrator.myIvyCalNode.IvySendSwitchBlock(myCalibrator.aircraftID,myCalibrator.landingBlockInteger)
time.sleep(2)
myCalibrator.killCopter()
myCalibrator.dataFile.saveToFile()
print("File saved")
myCalibrator.myIvyCalNode.SaveIvyKillLog()
logger.info("ProgramEnded")
try:
    myCalibrator.myIvyCalNode.IvyInitStop()
except:
    print("Ivy was already killed?")
raise SystemExit()
