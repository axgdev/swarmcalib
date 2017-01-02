#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  calibration.py
#

import finkenPID
import time
#To save the calibration parameters
import calibrationOutput

""" Logging imports """
import logging
#Ivy cal node
import ivyModules.IvyCalibrationNode
import math
import numpy

class Calibrator:
    """ Class to calibrate the copter, before starting it, it is
        important to set aircraftID, deadZone, basePosition (setPoint),
        and PollingTime

    """

    def __init__(self, logger):
    #Here we can put some default variables for deadzone, targetzone and pollingTime and PID parameters
        """PID parameters:"""
        self.pParameter = 0.015
        self.iParameter = 0
        self.dParameter = 0.001
        self.targetXController = finkenPID.PIDController(self.pParameter, self.iParameter, self.dParameter) #I set it to zero here for zero control
        self.targetYController = finkenPID.PIDController(self.pParameter, self.iParameter, self.dParameter)

        """Copter/GCS settings:"""
        self.copterTheta = 0;
        self.myIvyCalNode = ivyModules.IvyCalibrationNode.IvyCalibrationNode()
        self.airBlockInteger = 2 #actually start block, talk about naming things correctly ffs..
        self.emptyBlockInteger = 1
        self.landingBlockInteger = 4

        """Calibration routine constants and buffers:"""
        self.internalZoneSize = 150.0
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
        
        """Variables for output"""
        self.Xdiff = 0
        self.Ydiff = 0
        self.newPitch = 0
        self.newRoll = 0
        self.bestPitch = 0
        self.bestRoll = 0

        """Variables for output"""
        self.logger = logger
        self.totalIterations = 0
        self.initialTime = time.time()
        self.dataFile = calibrationOutput.CSVWriter()
        self.dataFile.setHeader(['currentTime', 'timeDifference', 'totalIterations',
                              'accumulateIter', 'calibIter', 'inInternalZone',
                              'copterXPos', 'copterYPos', 'copterTheta',
                              'errorX', 'errorY', 'Xdiff', 'Ydiff',
                              'accumulateX', 'accumulateY',
                              'newPitch', 'newRoll',
                              'bestPitch', 'bestRoll',
                              '-rollToSendRad', 'pitchToSendRad',
                              '-calRollToSendDeg', 'calPitchToSendDeg',
                              '-rollCalib', 'pitchCalib'])

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
        posList = self.myIvyCalNode.IvyGetPosList()
        self.copterXPos = posList[0]
        self.copterYPos = posList[1]
        self.copterTheta = posList[2]
        #self.logger.debug("X: "+str(self.copterXPos) + " Y: "+str(self.copterYPos) + " Theta: " + str(self.copterTheta))

    def killCopter(self):
        """Turns off the copter motors immediately through an Ivy message.
           We then wait a small amount of time and send the switch block
           empty to get the copter ready for the next flight.
           """
        self.logger.info("Copter Kill signal")
        self.myIvyCalNode.IvySendKill(self.aircraftID)
        time.sleep(0.1)
        self.myIvyCalNode.IvySendSwitchBlock(self.aircraftID,self.emptyBlockInteger)
        return

    def unkillCopter(self):
        """Gets the copter out of the KILL status
           """
        self.logger.info("Unkilling Copter")
        self.myIvyCalNode.IvySendUnKill(self.aircraftID)
        return

    def sendStartMode(self):
        """First it sets the empty block and then we put the copter in
            air mode to be able to fly the copter giving thrust with
            the remote control
        """
        self.logger.info("Sending start mode")
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

    def sendCalibrationToCopter(self,pitchCalib,rollCalib):
        """Interface function to Ivy python node. Integer 58 represents roll,
           and integer 59 represents pitch
        """
        self.myIvyCalNode.IvySendCalib(self.aircraftID, 58, rollCalib)
        self.myIvyCalNode.IvySendCalib(self.aircraftID, 59, pitchCalib)

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

    def outputData(self, errorX, errorY, rollToSend, pitchToSend):
        currentTime = time.time()
        timeDifference = currentTime - self.initialTime
        self.dataFile.append([currentTime, timeDifference, self.totalIterations,
                              self.accumulateIter, self.calibIter, self.inInternalZone,
                              self.copterXPos, self.copterYPos, self.copterTheta,
                              errorX, errorY, self.Xdiff, self.Ydiff,
                              self.accumulateX, self.accumulateY,
                              self.newPitch, self.newRoll,
                              self.bestPitch, self.bestRoll,
                              -rollToSend, pitchToSend,
                              -rollToSend*(math.pi/180), pitchToSend*(math.pi/180),
                              -self.rollCalib, self.pitchCalib])
        self.totalIterations += 1

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

        """Get output from the controllers based on the error we have"""
        pitchToSend = self.targetXController.step(errorX, self.pollingTime)
        rollToSend = self.targetYController.step(errorY,self.pollingTime)
        """ Send parameters to copter, our view of the roll is inverted
            what it should be on the copter, so we change the roll sign
        """
        self.sendParametersToCopter(pitchToSend, -rollToSend, 0)

        if (self.isInInternalZone(errorX,errorY)):

            if (self.inInternalZone == False):  #entering internal zone so reset the Iterator and position buffer
                self.accumulateIter = 0
            if (self.accumulateIter == 0):
                self.accumulateX = 0
                self.accumulateY = 0
                self.copterXOld = self.copterXPos
                self.copterYOld = self.copterYPos

            self.inInternalZone = True

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
                self.logger.debug("movement over last 100 iterations: " +str(self.Xdiff + self.Ydiff))
                if (90 > (self.Xdiff + self.Ydiff)):
                    """movement below treshold for 100 iterations,
                       use accumulated PID values for calibration
                       """
                    """ Saving calibration parameters to file. No worries this
                        is non blocking call, it runs in separate thread.
                        Important to save it everytime just in case the app is
                        interrumpted, uncomment if you want to use it
                    """
                    """
                    calibrationOutput.saveCalibration(self.bestPitch,self.bestRoll,self.absDiff,"pitch","roll","diff")
                    """
                    self.newPitch = self.accumulateX/100
                    self.newRoll = self.accumulateY/100
                    self.bestPitch = (self.bestPitch + self.newPitch)
                    self.bestRoll = (self.bestRoll + self.newRoll)
                    self.calibIter += 1


                    self.logger.debug("new calib values: Roll: " +str(-self.newRoll) + "  Pitch: " + str(self.newPitch))
                    self.logger.debug("total calib values: Roll: " +str(-self.bestRoll/self.calibIter) + "  Pitch: " + str(self.newPitch/self.calibIter))

                    """Incrementally increase the calibration
                       values of the copter, disregard first 2
                       iterations for stability
                       """
                    if (self.calibIter > 0):
                        self.rollCalib += 0.025*self.newRoll
                        self.pitchCalib += 0.025*self.newPitch
                        self.sendCalibrationToCopter(self.pitchCalib,-self.rollCalib)
                        self.logger.debug("incremental calib values #" + str(self.calibIter) + ": Roll: " +str(-self.rollCalib) + "  Pitch: " + str(self.pitchCalib))
                        self.logger.debug("Camera parameters X:" + str(self.copterXPos) + " Y:" + str(self.copterYPos))
        elif (self.inInternalZone):
            self.inInternalZone = False
        self.outputData(errorX, errorY, rollToSend, pitchToSend)
