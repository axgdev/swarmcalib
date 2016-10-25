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

class Calibrator:
    """ Class to calibrate the copter, before starting it, it is 
        important to set aircraftID, deadZone, basePosition (setPoint), 
        and PollingTime
    
    """

    def __init__(self, logger):
    #Here we can put some default variables for deadzone, targetzone and pollingTime and PID parameters
        self.pParameter = 0.015
        self.iParameter = 0
        self.dParameter = 0.0015
        self.iParameter=0
        self.targetXController = finkenPID.PIDController(self.pParameter, self.iParameter, self.dParameter) #I set it to zero here for zero control
        self.targetYController = finkenPID.PIDController(self.pParameter, self.iParameter, self.dParameter)
        self.copterXPos = 1 #Just to test
        self.copterYPos = 1 #Just to test
        self.copterTheta = 0;
        self.calibrationParameters = [0,0]
        self.myIvyCalNode = ivyModules.IvyCalibrationNode.IvyCalibrationNode()
        self.airBlockInteger = 2
        self.emptyBlockInteger = 1
        self.landingBlockInteger = 4
        self.internalZoneSize = 75.0
        self.inInternalZone = False
        self.inExternalZone = True
        self.accumulateX = 0.0
        self.accumulateY = 0.0
        self.accumulateIter = 0
        self.accumulateXAvg = 0
        self.accumulateYAvg = 0
        self.bestPitch = 0
        self.bestRoll = 0
        self.copterXOld = 0
        self.copterYOld = 0
        self.absDiff = 1000
        self.calibIter = 0
        self.logger = logger
    
    #Important INIT
    def setBasePosition(self, posX, posY):
        """ PID Setpoint. Where the copter should be. In theory this point
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
    
    #Important INIT
    def setDeadZone(self, minX, maxX, minY, maxY):
        """ This defines the area where the copter should be killed. It
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

    #Important INIT
    def setPollingTime(self, pollingTime):
        """ For the PID parameters we need to keep track of the changes
            of the error throughout the time. We set here that discrete
            time of the PID loop.
            
            Parameters
            ----------
            pollingTime: float value
                Time in seconds. Example: pollingTime=1 is 1000ms. 
                pollingTime=0.1 is 100ms
        
        """
        self.pollingTime = pollingTime
        
    #Important INIT
    def setAircraftID(self, aircraftID):
        """ Every time we flash the copter in Paparazzi, we set an
            aircraftID, this is important to send the messages through
            Ivy as we need this aircraftID
            
            Parameters
            ----------
            aircraftID: integer value
        
        """
        self.aircraftID = aircraftID
        
    def getXYCoordinates(self):
        """ Calls the ivy node and the ivy node calls ROS to get the 
            position values from the tracking software, then we save them 
            in the class variables copterXPos, copterYPos and copterTheta
        
        """
        myObj = self.myIvyCalNode.IvyGetPos()
        self.copterXPos = myObj.x
        self.copterYPos = myObj.y
        self.copterTheta = myObj.theta
        #self.logger.debug("X: "+str(self.copterXPos) + " Y: "+str(self.copterYPos) + " Theta: " + str(self.copterTheta))
         
    def killCopter(self):
        """ Turns off the copter motors immediately through an Ivy message.
            We then wait a small amount of time and send the switch block
            empty to get the copter ready for the next flight.
        
        """
        self.logger.info("Copter Kill signal")
        self.myIvyCalNode.IvySendKill(self.aircraftID)
        time.sleep(0.1)
        self.myIvyCalNode.IvySendSwitchBlock(self.aircraftID,self.emptyBlockInteger)
        return
        
    def unkillCopter(self):
        """ Gets the copter out of the KILL status
        
        """
        self.logger.info("Unkilling Copter")
        self.myIvyCalNode.IvySendUnKill(self.aircraftID)
        return
        
    def sendStartMode(self):
        """ First it sets the empty block and then we put the copter in
            air mode to be able to fly the copter giving thrust with
            the remote control
        
        """
        self.logger.info("Sending start mode")
        self.myIvyCalNode.IvySendSwitchBlock(self.aircraftID,self.emptyBlockInteger)
        time.sleep(0.1)
        self.myIvyCalNode.IvySendSwitchBlock(self.aircraftID,self.airBlockInteger)
        
    def sendParametersToCopter(self, pitchToSend, rollToSend, yawToSend):
        """ Parameters to control the pitch, roll and yaw of the copter.
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
        #self.logger.debug("roll: "+str(rollToSend)+" pitch: "+str(pitchToSend))
        #self.logger.info("\n") #A new line for nicer output.
        self.myIvyCalNode.IvySendCalParams(self.aircraftID, 0, rollToSend, pitchToSend, yawToSend)
        return
        

        
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
        
        """ if we are in safe zone start another PID controller to control
             calibration parameters and we reset the other controllers
             note that inInternalZone is a boolean to know when we go
             in and out of the zone
        """
        if (self.isInInternalZone(errorX,errorY)):
            #self.logger.debug("in safe zone")
            if (self.inInternalZone == False):
                self.sendParametersToCopter(0, -0, 0)
                #self.myIvyCalNode.IvySendCalib(self.aircraftID, 58, -self.bestRoll)
                #self.myIvyCalNode.IvySendCalib(self.aircraftID, 59, self.bestPitch)
                """
                self.targetXController.p /= 4
                self.targetXController.i /= 4
                self.targetYController.p /= 4
                self.targetYController.i /= 4
                """
                self.accumulateIter = 0
                
            if (self.accumulateIter == 0):  #reset positioning memory and calib average
                self.accumulateX = 0
                self.accumulateY = 0
                self.copterXOld = self.copterXPos
                self.copterYOld = self.copterYPos
            self.inInternalZone = True          
             
            rollToSend = self.targetXController.step(errorY,self.pollingTime)
            pitchToSend = self.targetYController.step(errorX, self.pollingTime)            
            #self.myIvyCalNode.IvySendCalib(self.aircraftID, 58, -calRollToSend)
            #self.myIvyCalNode.IvySendCalib(self.aircraftID, 59, calPitchToSend)
            self.sendParametersToCopter(pitchToSend, -rollToSend, 0)
            #self.logger.debug("Sending calib pitch in degrees>: %f / roll %f" % (pitchToSend, -rollToSend))
            calRollToSend=rollToSend*(math.pi/180)
            calPitchToSend=pitchToSend*(math.pi/180)
            
            #self.logger.debug("Sending calib pitch: %f / roll %f" % (calPitchToSend, -calRollToSend))
            self.accumulateX = self.accumulateX + calPitchToSend
            self.accumulateY = self.accumulateY + calRollToSend
            self.accumulateIter += 1
            if (self.accumulateIter >= 100):
                #self.logger.debug("calculating movement..")
                self.accumulateIter = 0
                self.logger.debug("movement last iteration: " +str(self.absDiff))
                self.Xdiff = math.fabs(self.copterXPos - self.copterXOld)
                self.Ydiff = math.fabs(self.copterYPos - self.copterYOld)
                self.logger.debug("movement this iteration: " +str(self.Xdiff + self.Ydiff))
                if (75 > (self.Xdiff + self.Ydiff)):                    
                    self.absDiff = self.Xdiff + self.Ydiff
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
                    self.bestRoll = (self.bestPitch + self.newPitch)
                    self.calibIter += 1
                    
                    
                    self.logger.debug("new calib values: Roll: " +str(-self.newRoll/self.calibIter) + "  Pitch: " + str(self.newPitch/self.calibIter))
                    self.logger.debug("total calib values: Roll: " +str(-self.bestRoll/self.calibIter) + "  Pitch: " + str(self.newPitch/self.calibIter))         
                    #self.myIvyCalNode.IvySendCalib(self.aircraftID, 58, -self.bestRoll)
                    #self.myIvyCalNode.IvySendCalib(self.aircraftID, 59, self.bestPitch)
                    self.sendParametersToCopter(0, -0, 0)
                    self.targetXController.reset()
                    self.targetYController.reset()
                self.accumulateIter = 0
                
                
            return
        elif (self.inInternalZone):                
            self.inInternalZone = False
            #self.logger.debug("Exiting internal zone")
            """
            self.targetXController.p *= 4
            self.targetXController.i *= 4
            self.targetYController.p *= 4
            self.targetYController.i *= 4
            """
        #self.logger.debug('ErrorX: '+str(errorX)+' ErrorY: '+str(errorY))

        """ Get output from the controllers based on the error we have """
        rollToSend = self.targetYController.step(errorY, self.pollingTime)
        pitchToSend = self.targetXController.step(errorX, self.pollingTime)
        """ Send parameters to copter, our view of the roll is inverted
            what it should be on the copter, so we change the roll sign """
        self.sendParametersToCopter(pitchToSend, -rollToSend, 0)
        #Save in list constantly
        #self.calibrationParameters = [pitchToSend, -rollToSend]
        
    def isInInternalZone(self,errorX,errorY):
        return (math.fabs(errorX) < self.internalZoneSize) and (math.fabs(errorY) < self.internalZoneSize)
        
    def isInDeadZone(self):
        """ Method to see if copter is outside of the safe zone to fly
        
            Returns
            -------
            boolean value
                True if copter is in deadZone, false if its not
        
        """
        if ((self.copterXPos > self.maxX) or (self.copterXPos < self.minX) 
            or (self.copterYPos > self.maxY) or(self.copterYPos < self.minY)):
            return True
        return False