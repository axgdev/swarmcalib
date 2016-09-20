#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  calibration.py
#

import finkenPID
import time
#import json  #Remove unused module
import datetime
#To save the calibration parameters
import calibrationOutput

""" Logging imports """
import logging
import os

#ROS libraries... maybe the communication python files should have it
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
#Ivy cal node
import ivyModules.IvyCalibrationNode
import math
import numpy

""" Superior logging capabilities. Way better than prints :P """

#Set the logger object with the name we have chosen
logger = logging.getLogger('cal')
#Set the logger level, for all cases. This will be configured for each handler
logger.setLevel(logging.DEBUG)

#Create the log folder if it does not exist
if not os.path.exists("logs"):
    os.makedirs("logs")

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

class Calibrator:
    """ Class to calibrate the copter, before starting it, it is
        important to set aircraftID, deadZone, basePosition (setPoint),
        and PollingTime

    """

    def __init__(self):
    #Here we can put some default variables for deadzone, targetzone and pollingTime and PID parameters
        """PID parameters:"""
        self.pParameter = 0.015
        self.iParameter = 0
        self.dParameter = 0.0015
        self.targetXController = finkenPID.PIDController(self.pParameter, self.iParameter, self.dParameter) #I set it to zero here for zero control
        self.targetYController = finkenPID.PIDController(self.pParameter, self.iParameter, self.dParameter)

        """Copter/GCS settings:"""
        self.copterTheta = 0;
        self.myIvyCalNode = ivyModules.IvyCalibrationNode.IvyCalibrationNode()
        self.airBlockInteger = 2 #actually start block, talk about naming things correctly ffs..
        self.emptyBlockInteger = 1
        self.landingBlockInteger = 4

        """calibration routine constants and buffers:"""
        self.internalZoneSize = 75.0
        self.inInternalZone = False
        self.inExternalZone = True
        self.accumulateX = 0.0
        self.accumulateY = 0.0
        self.accumulateIter = 0
        self.bestPitch = 0
        self.bestRoll = 0
        self.copterXOld = 0
        self.copterYOld = 0
        self.calibIter = 0
        self.rollCalib = 0
        self.pitchCalib = 0


    def setBasePosition(self, posX, posY):
        """PID Setpoint. Where the copter should be. In theory this point
           could be moved every certain time and make the copter follow
           a pattern, however here in this calibration point we just set
           one point and keep it throughout all the calibration routine
           Parameters
           ----------
           posX: float value
                 The X position we want the copter to be in
           posY: float value
                 The Y position we want the copter to be in
           """
        self.baseX = posX
        self.baseY = posY

    def setDeadZone(self, minX, maxX, minY, maxY):
        """This defines the area where the copter should be killed. It
           should be killed either because the copter is unstable (close
           to walls) or because the camera detection is unreliable.
           Parameters
           ----------
           minX,maxX,minY,maxY: float value
                These represent a rectangle. Outside of this rectangle is
                the deadZone. We do a check with the function isInDeadZone
           """
        self.minX = minX
        self.maxX = maxX
        self.minY = minY
        self.maxY = maxY

    def setPollingTime(self, pollingTime):
        """For the PID parameters we need to keep track of the changes
           of the error throughout the time. We set here that discrete
           time of the PID loop.
           Parameters
           ----------
           pollingTime: float value
                Time in seconds. Example: pollingTime=1 is 1000ms.
                pollingTime=0.1 is 100ms
           """
        self.pollingTime = pollingTime

    def setAircraftID(self, aircraftID):
        """Every time we flash the copter in Paparazzi, we set an
           aircraftID, this is important to send the messages through
           Ivy as we need this aircraftID
           Parameters
           ----------
           aircraftID: integer value
        """
        self.aircraftID = aircraftID


"""IVY FUNCTIONS:"""
    def getXYCoordinates(self):
        """ Calls the ivy node and the ivy node calls ROS to get the
            position values from the tracking software, then we save them
            in the class variables copterXPos, copterYPos and copterTheta
            """
        myObj = self.myIvyCalNode.IvyGetPos()
        self.copterXPos = myObj.x
        self.copterYPos = myObj.y
        self.copterTheta = myObj.theta
        #logger.debug("X: "+str(self.copterXPos) + " Y: "+str(self.copterYPos) + " Theta: " + str(self.copterTheta))

    def killCopter(self):
        """Turns off the copter motors immediately through an Ivy message.
           We then wait a small amount of time and send the switch block
           empty to get the copter ready for the next flight.
           """
        logger.info("Copter Kill signal")
        self.myIvyCalNode.IvySendKill(self.aircraftID)
        time.sleep(0.1)
        self.myIvyCalNode.IvySendSwitchBlock(self.aircraftID,self.emptyBlockInteger)
        return

    def unkillCopter(self):
        """Gets the copter out of the KILL status
           """
        logger.info("Unkilling Copter")
        self.myIvyCalNode.IvySendUnKill(self.aircraftID)
        return

    def sendStartMode(self):
        """First it sets the empty block and then we put the copter in
            air mode to be able to fly the copter giving thrust with
            the remote control
        """
        logger.info("Sending start mode")
        self.myIvyCalNode.IvySendSwitchBlock(self.aircraftID,self.emptyBlockInteger)
        time.sleep(0.1)
        self.myIvyCalNode.IvySendSwitchBlock(self.aircraftID,self.airBlockInteger)

    def sendParametersToCopter(self, pitchToSend, rollToSend, yawToSend):
        """Parameters to control the pitch, roll and yaw of the copter.
           It is used by the calibration to bring back the copter to
           the set point. In Paparazzi, these values are added to the
           remote control input, meaning that we can control the copter
           using the copter, and we could use the remote if necessary to
           avoid some heavy crash of the copter or something similar.
           Parameters
           ----------
           pitch, roll, yaw: float values
                Use this values to make the copter move in a certain
                direction
           """
        self.myIvyCalNode.IvySendCalParams(self.aircraftID, 0, rollToSend, pitchToSend, yawToSend)
        return


"""Auxiliary functions for the calibration routine:"""
    def isInInternalZone(self,errorX,errorY):
        return (math.fabs(errorX) < self.internalZoneSize) and (math.fabs(errorY) < self.internalZoneSize)

    def isInDeadZone(self):
        """Method to see if copter is outside of the safe zone to fly
           Returns
           -------
           boolean value
              True if copter is in deadZone, false if its not
           """
        if ((self.copterXPos > self.maxX) or (self.copterXPos < self.minX)
            or (self.copterYPos > self.maxY) or(self.copterYPos < self.minY)):
            return True
        return False

"""Main Calibration logic:"""
    def followTarget(self):
        """ One of the main functions of the calibration. Once we have
            the position of the copter and setPoint (basePosition) we
            calculate the error which is the difference between the
            copter position and the setPoint, then we proceed to calculate
            the new parameters we have to send to the copter making a PID
            step.

        """
        """ Calculate error as difference from copter pos and setPoint pos """
        errorX = (self.copterXPos - self.baseX)
        errorY = (self.copterYPos - self.baseY)

        """ Do axis transformation using transformation matrix
            from what the camera sees to what the copter sees """
        coord = numpy.array([errorX,errorY])
        self.copterTheta=self.copterTheta*math.pi/180
        translationMatrix = numpy.matrix([[math.cos(self.copterTheta), math.sin(self.copterTheta)], [-math.sin(self.copterTheta), math.cos(self.copterTheta)]])
        newCoord = numpy.dot(translationMatrix,coord)
        """ Overwrite the old errors with transformed errors """
        errorX = newCoord.item(0)
        errorY = newCoord.item(1)

        if (self.isInInternalZone(errorX,errorY)):

            if (self.inInternalZone == False):  #entering internal zone so reset the Iterator and position buffer
                self.accumulateIter = 0
                self.accumulateX = 0
                self.accumulateY = 0
                self.copterXOld = self.copterXPos
                self.copterYOld = self.copterYPos
            self.inInternalZone = True

            rollToSend = self.targetXController.step(errorY,self.pollingTime)
            pitchToSend = self.targetYController.step(errorX, self.pollingTime)
            self.sendParametersToCopter(pitchToSend, -rollToSend, 0)

            """Transform from degrees to rad"""
            calRollToSend=rollToSend*(math.pi/180)
            calPitchToSend=pitchToSend*(math.pi/180)

            """Accumulate PID corrections for later use in calibration"""
            self.accumulateX = self.accumulateX + calPitchToSend
            self.accumulateY = self.accumulateY + calRollToSend
            self.accumulateIter += 1


            if (self.accumulateIter >= 100):
                """after 100 iterations, check absolute movement and,
                   if low enough, calibrate
                   """
                self.accumulateIter = 0
                self.Xdiff = math.fabs(self.copterXPos - self.copterXOld)
                self.Ydiff = math.fabs(self.copterYPos - self.copterYOld)
                logger.debug("movement over last 100 iterations: " +str(self.Xdiff + self.Ydiff))
                if (75 > (self.Xdiff + self.Ydiff)):
                    """movement below treshold for 100 iterations,
                       use accumulated PID values for calibration
                       """
                    """ Saving calibration parameters to file. No worries this 
                        is non blocking call, it runs in separate thread.
                        Important to save it everytime just in case the app is
                        interrumpted
                    """
                    calibrationOutput.saveCalibration(self.bestPitch,self.bestRoll,self.absDiff,"pitch","roll","diff")
                    self.newPitch = self.accumulateX/100
                    self.newRoll = self.accumulateY/100
                    self.bestPitch = (self.bestPitch + self.newPitch)
                    self.bestRoll = (self.bestPitch + self.newPitch)
                    self.calibIter += 1


                    logger.debug("new calib values: Roll: " +str(-self.newRoll) + "  Pitch: " + str(self.newPitch))
                    logger.debug("total calib values: Roll: " +str(-self.bestRoll/self.calibIter) + "  Pitch: " + str(self.newPitch/self.calibIter))

                    """Incrementally increase the calibration
                       values of the copter, disregard first 2
                       iterations for stability
                       """
                    if (calibIter > 2):
                        self.rollCalib += 0.05*newRoll
                        self.pitchCalib += 0.05*newPitch
                        self.myIvyCalNode.IvySendCalib(self.aircraftID, 58, -rollCalib)
                        self.myIvyCalNode.IvySendCalib(self.aircraftID, 59, pitchCalib)
                        logger.debug("incremental calib values: Roll: " +str(-self.rollCalib) + "  Pitch: " + str(self.pitchCalib))

                self.accumulateIter = 0


            return
        elif (self.inInternalZone):
            self.inInternalZone = False

        """Get output from the controllers based on the error we have"""
        rollToSend = self.targetYController.step(errorY, self.pollingTime)
        pitchToSend = self.targetXController.step(errorX, self.pollingTime)
        """ Send parameters to copter, our view of the roll is inverted
            what it should be on the copter, so we change the roll sign
            """
        self.sendParametersToCopter(pitchToSend, -rollToSend, 0)



"""Initialization for main function"""
myCalibrator = Calibrator()
myCalibrator.setDeadZone(250,1250,250,950) #minX, maxX, minY, maxY
myCalibrator.setBasePosition(750,600)
myCalibrator.setPollingTime(0.005) #optimum: 0.005
myCalibrator.setAircraftID(5)


""" Reading previous best calibration parameters """
inputParams = calibrationOutput.loadCalibration()
if (len(inputParams) > 1)
    myCalibrator.bestPitch = inputParams[0]*math.pi/180
    myCalibrator.bestRoll = inputParams[1]*math.pi/180
    myCalibrator.absDiff = inputParams[2]*math.pi/180

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
myCalibrator.unkillCopter()
time.sleep(3) #For the camera to detect the initial position
myCalibrator.sendStartMode() #I uncommented this for simulation purposes
time.sleep(1.75) #When the copter turns on, there are no lights until a few seconds

i = 0;
while(myCalibrator.calibIter < 22):
    myCalibrator.getXYCoordinates()
    if (myCalibrator.isInDeadZone()):
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

if (myCalibrator.calibIter != 0):
    myCalibrator.bestRoll /= myCalibrator.calibIter
    myCalibrator.bestPitch /= myCalibrator.calibIter
myCalibrator.sendParametersToCopter(0, 0, 0)
myCalibrator.myIvyCalNode.IvySendCalib(myCalibrator.aircraftID, 58, -myCalibrator.bestRoll)
myCalibrator.myIvyCalNode.IvySendCalib(myCalibrator.aircraftID, 59, myCalibrator.bestPitch)
logger.debug("final calib values: Roll: " +str(-myCalibrator.bestRoll) + "  Pitch: " + str(myCalibrator.bestPitch) + " Calib iter: " + str(myCalibrator.calibIter))       
time.sleep(1)
myCalibrator.myIvyCalNode.IvySendSwitchBlock(myCalibrator.aircraftID,myCalibrator.landingBlockInteger)
time.sleep(2)
myCalibrator.killCopter()
logger.info("ProgramEnded")
raise SystemExit()
