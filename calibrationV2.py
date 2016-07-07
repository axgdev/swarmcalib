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
#from os import walk #Remove unused module

#ROS libraries... maybe the communication python files should have it
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
#Ivy cal node
import ivyModules.IvyCalibrationNode

class Calibrator:
    """Class to calibrate the copter"""

    
    def __init__(self):
    #Here we can put some default variables for deadzone, targetzone and pollingTime and PID parameters
        self.targetXController = finkenPID.PIDController(4, 0, 4)
        self.targetYController = finkenPID.PIDController(4, 0, 4)
        self.copterXPos = 1 #Just to test
        self.copterYPos = 1 #Just to test
        self.calibrationParameters = [0,0]
        self.myIvyCalNode = IvyCalibrationNode()
    
    #Important INIT
    def setBasePosition(self, posX, posY):
        self.baseX = posX
        self.baseY = posY
    
    #Important INIT
    def setDeadZone(self, minX, maxX, minY, maxY):
        self.minX = minX
        self.maxX = maxX
        self.minY = minY
        self.maxY = maxY

    #Important INIT
    def setPollingTime(self, pollingTime):
        self.pollingTime = pollingTime
        
    #Important INIT
    def setAircraftID(self, aircraftID):
        self.aircraftID = aircraftID
        
    def getXYCoordinates(self):
         #Call here the ivy method, it should return XY coordinates
         #We will set here 
         #self.copterXPos
         #self.copterYPos
         myObj = myIvyCalNode.IvyGetPos()
         self.copterXPos = myObj.x
         self.copterYPos = myObj.y
         print("X: "+str(self.copterXPos))
         print("Y: "+str(self.copterYPos))
         
    def killCopter(self):
        #Call kill copter command
        print("Copter Kill signal")
        self.myIvyCalNode.IvySendKill(self.aircraftID)
        return
        
    def unkillCopter(self):
        print("Unkilling Copter")
        self.myIvyCalNode.IvySendUnKill(self.aircraftID)
        return
        
    def sendStartMode(self):
        print("Sending start mode")
        self.myIvyCalNode.IvySendStartBlock(self.aircraftID)
        
    def sendPitch(self, pitchToSend):
        #Call send pitch
        return
        
    def sendRoll(self, rollToSend):
        #Call send roll
        return
        
    def sendParametersToCopter(self, pitchToSend, rollToSend, yawToSend):
        #IvyCalibrationNode.IvySendParams
        print("Parameters sent")
        return
        
    def sendYaw(self):
        #Call send yaw, I think this one is not required.
        return
        
    def followTarget(self):
        errorX = self.copterXPos - self.baseX
        errorY = self.copterYPos - self.baseY
        pitchToSend = self.targetXController.step(errorX, self.pollingTime)
        rollToSend = self.targetYController.step(errorY, self.pollingTime)
        #self.sendPitch(pitchToSend)
        #self.sendRoll(rollToSend)
        self.sendParametersToCopter(pitchToSend, rollToSend, 0)
        #Save in list constantly
        self.calibrationParameters = [pitchToSend, rollToSend] 
        
    def isInDeadZone(self):
        if ((self.copterXPos > self.maxX) or (self.copterXPos < self.minX) 
            or (self.copterYPos > self.maxY) or(self.copterYPos < self.minY)):
            self.killCopter();
            return True;
        return False;

myCalibrator = Calibrator()
myCalibrator.setDeadZone(-0.48,1.7,-0.69,2.70) #minX, maxX, minY, maxY
myCalibrator.setBasePosition(0,0)
myCalibrator.setPollingTime(0.5)
myCalibrator.setAircraftID(5)
myCalibrator.myIvyCalNode.IvyInitStart()
myCalibrator.unkillCopter()
myCalibrator.sendStartMode()
while(True):
    myCalibrator.getXYCoordinates()
    if (myCalibrator.isInDeadZone()):
        myCalibrator.killCopter()
        #save calibration parameters.. the filename will be a timestamp
        calibrationOutput.saveObject(myCalibrator.calibrationParameters,'')
        myCalibrator.myIvyCalNode.IvyInitStop()
        break;
    myCalibrator.followTarget()
    time.sleep(myCalibrator.pollingTime)
    
print("ProgramEnded")





