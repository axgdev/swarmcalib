#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  calibration.py
#  
#  Copyright 2016  <>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  

import finkenPID
import time
#import IvyCalibrationNode

class Calibrator:
    'Class to calibrate the copter'
    
    def __init__(self):
    #Here we can put some default variables for deadzone, targetzone and pollingTime and PID parameters
        self.targetXController = finkenPID.PIDController(4, 0, 4)
        self.targetYController = finkenPID.PIDController(4, 0, 4)
        self.copterXPos = 1 #Just to test
        self.copterYPos = 1 #Just to test
    
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
        
    def getXYCoordinates(self):
         #Call here the ivy method, it should return XY coordinates
         #We will set here 
         #self.copterXPos
         #self.copterYPos
         self.copterXPos += 0.1
         self.copterYPos -= 0.1
         print("X: "+str(self.copterXPos))
         print("Y: "+str(self.copterYPos))
         
    def killCopter(self):
        #Call kill copter command
        return
        
    def sendPitch(self, pitchToSend):
        #Call send pitch
        return
        
    def sendRoll(self, rollToSend):
        #Call send roll
        return
        
    def sendParametersToCopter(self, pitchToSend, rollToSend, yawToSend):
        #IvyCalibrationNode.IvySendParams
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
        self.sendParametersToCopter(pitchToSend,rollToSend,0)
        
    def checkForDeadZone(self):
        if ((self.copterXPos > self.maxX) or (self.copterXPos < self.minX) 
            or (self.copterYPos > self.maxY) or(self.copterYPos < self.minY)):
            self.killCopter();
            return True;
        return False;

myCalibrator = Calibrator()
myCalibrator.setDeadZone(-0.48,1.7,-0.69,2.70) #minX, maxX, minY, maxY
myCalibrator.setBasePosition(0,0)
myCalibrator.setPollingTime(0.5)
while(True):
    myCalibrator.getXYCoordinates()
    if (myCalibrator.checkForDeadZone()):
        print("Copter Kill signal")
        break;
    myCalibrator.followTarget()
    time.sleep(myCalibrator.pollingTime)
    
print("ProgramEnded")





