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

    def __init__(self):
    #Here we can put some default variables for deadzone, targetzone and pollingTime and PID parameters
        self.targetXController = finkenPID.PIDController(0.0175, 0.0000001, 0) #I set it to zero here for zero control
        self.targetYController = finkenPID.PIDController(0.0175, 0.0000001, 0)
        self.copterXPos = 1 #Just to test
        self.copterYPos = 1 #Just to test
        self.copterTheta = 0;
        self.calibrationParameters = [0,0]
        self.myIvyCalNode = ivyModules.IvyCalibrationNode.IvyCalibrationNode()
        self.airBlockInteger = 2
        self.emptyBlockInteger = 1
        self.landingBlockInteger = 4
        self.internalZoneSize = 50.0
        self.externalZoneSize = 75.0
        self.inInternalZone = False
        self.inExternalZone = True
        self.accumulateX = 0.0
        self.accumulateY = 0.0
        self.accumulateIter = 0
        self.accumulateXAvg = 0
        self.accumulateYAvg = 0
    
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
        print("X: "+str(self.copterXPos) + " Y: "+str(self.copterYPos) + " Theta: " + str(self.copterTheta))
         
    def killCopter(self):
        """ Turns off the copter motors immediately through an Ivy message.
            We then wait a small amount of time and send the switch block
            empty to get the copter ready for the next flight.
        
        """
        print("Copter Kill signal")
        self.myIvyCalNode.IvySendKill(self.aircraftID)
        time.sleep(0.1)
        self.myIvyCalNode.IvySendSwitchBlock(self.aircraftID,self.emptyBlockInteger)
        return
        
    def unkillCopter(self):
        """ Gets the copter out of the KILL status
        
        """
        print("Unkilling Copter")
        self.myIvyCalNode.IvySendUnKill(self.aircraftID)
        return
        
    def sendStartMode(self):
        """ First it sets the empty block and then we put the copter in
            air mode to be able to fly the copter giving thrust with
            the remote control
        
        """
        print("Sending start mode")
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
        print("roll: "+str(rollToSend)+" pitch: "+str(pitchToSend))
        print() #A new line for nicer output.
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

        """Error Accumulation if in safe zone"""
        if (math.fabs(errorX) < self.internalZoneSize) and (math.fabs(errorY) < self.internalZoneSize):
        print("entering internal")
        self.inInternalZone = True
            self.accumulateX = self.accumulateX + errorX
        self.accumulateY = self.accumulateY + errorY
        self.accumulateIter = self.accumulateIter + 1
        elif (self.inInternalZone == True):
        print("exiting internal Zone")
            self.accumulateXAvg =(self.accumulateX/self.accumulateIter)*self.targetXController.p/10
        self.accumulateYAvg =(self.accumulateY/self.accumulateIter)*self.targetYController.p/10
        print('accuX' + str(self.accumulateX) + ' accuIter:' +str(self.accumulateIter))
        self.inInternalZone = False
        #set Pitch calib
            print("setting pitch")
            self.myIvyCalNode.IvySendCalib(self.aircraftID, 59, self.accumulateXAvg)
        #set Roll Calib
        self.myIvyCalNode.IvySendCalib(self.aircraftID, 58, -self.accumulateYAvg)

        print('ErrorX: '+str(errorX)+' ErrorY: '+str(errorY))

        if (math.fabs(errorX) <= self.internalZoneSize):
            errorX = 0
        if (math.fabs(errorY) <= self.internalZoneSize):
            errorY = 0

        print('ErrorX: '+str(errorX)+' ErrorY: '+str(errorY))

        """ Get output from the controllers based on the error we have """
        rollToSend = self.targetXController.step(errorY, self.pollingTime)
        pitchToSend = self.targetYController.step(errorX, self.pollingTime)
        """ Send parameters to copter, our view of the roll is inverted
            what it should be on the copter, so we change the roll sign """
        self.sendParametersToCopter(pitchToSend, -rollToSend, 0)
        #Save in list constantly
        self.calibrationParameters = [pitchToSend, -rollToSend] 
        
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

myCalibrator = Calibrator()
#myCalibrator.setDeadZone(-0.2,1.0,-0.1,2.1) #minX, maxX, minY, maxY
myCalibrator.setDeadZone(250,1250,250,950) #minX, maxX, minY, maxY
myCalibrator.setBasePosition(750,600)
myCalibrator.setPollingTime(0.005) #optimum: 0.005
myCalibrator.setAircraftID(5)
myCalibrator.myIvyCalNode.IvyInitStart()
myCalibrator.sendParametersToCopter(0, 0, 0) #We make sure pitch, roll and yaw are 0 at start
myCalibrator.unkillCopter()
time.sleep(3) #For the camera to detect the initial position
myCalibrator.sendStartMode() #I uncommented this for simulation purposes
time.sleep(1.75) #When the copter turns on, there are no lights until a few seconds
i = 0;
while(i<=10000000):
    myCalibrator.getXYCoordinates()
    if (myCalibrator.isInDeadZone()):
        myCalibrator.killCopter()
        #save calibration parameters.. the filename will be a timestamp
        calibrationOutput.saveObject(myCalibrator.calibrationParameters,'')
        myCalibrator.myIvyCalNode.IvyInitStop()
        break;
    #time.sleep(3)
    myCalibrator.followTarget()
    i=i+1
    time.sleep(myCalibrator.pollingTime)
    
myCalibrator.myIvyCalNode.IvySendSwitchBlock(myCalibrator.aircraftID,myCalibrator.landingBlockInteger)
time.sleep(2)
myCalibrator.killCopter()
print("ProgramEnded")
raise SystemExit()
