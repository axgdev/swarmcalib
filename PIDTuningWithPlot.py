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
import matplotlib.pyplot as plt

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
        self.pParameter = 2.0/100
        self.iParameter = 1.0/10000000
        self.dParameter = 0.0
        self.targetXController = finkenPID.PIDController(self.pParameter, self.iParameter, self.dParameter) #I set it to zero here for zero control
        self.targetYController = finkenPID.PIDController(self.pParameter, self.iParameter, self.dParameter)
        self.internalXController = finkenPID.PIDController(0.005, 0.0000001/4, 0)
        self.internalYController = finkenPID.PIDController(0.005, 0.0000001/4, 0)
        self.copterXPos = 1 #Just to test
        self.copterYPos = 1 #Just to test
        self.copterTheta = 0;
        self.calibrationParameters = [0,0]
        self.myIvyCalNode = ivyModules.IvyCalibrationNode.IvyCalibrationNode()
        self.airBlockInteger = 2
        self.emptyBlockInteger = 1
        self.landingBlockInteger = 4
        self.accumulatedTime = 0.0
    
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
        logger.debug("X: "+str(self.copterXPos) + " Y: "+str(self.copterYPos) + " Theta: " + str(self.copterTheta))
         
    def killCopter(self):
        """ Turns off the copter motors immediately through an Ivy message.
            We then wait a small amount of time and send the switch block
            empty to get the copter ready for the next flight.
        
        """
        logger.info("Copter Kill signal")
        self.myIvyCalNode.IvySendKill(self.aircraftID)
        time.sleep(0.1)
        self.myIvyCalNode.IvySendSwitchBlock(self.aircraftID,self.emptyBlockInteger)
        return
        
    def unkillCopter(self):
        """ Gets the copter out of the KILL status
        
        """
        logger.info("Unkilling Copter")
        self.myIvyCalNode.IvySendUnKill(self.aircraftID)
        return
        
    def sendStartMode(self):
        """ First it sets the empty block and then we put the copter in
            air mode to be able to fly the copter giving thrust with
            the remote control
        
        """
        logger.info("Sending start mode")
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
        logger.debug("roll: "+str(rollToSend)+" pitch: "+str(pitchToSend))
        logger.info("\n") #A new line for nicer output.
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
        
        """ Plotting part """
        """ Roll """
        self.accumulatedTime++
        plt.figure(1)
        plt.scatter(self.accumulatedTime,errorY)
        plt.pause(0.02)
        
        """ Pitch """
		plt.figure(2)
		plt.scatter(self.accumulatedTime,errorX)
		plt.pause(0.02)
        
        logger.debug('ErrorX: '+str(errorX)+' ErrorY: '+str(errorY))

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

myCalibrator = Calibrator()
#myCalibrator.setDeadZone(-0.2,1.0,-0.1,2.1) #minX, maxX, minY, maxY
myCalibrator.setDeadZone(250,1250,250,950) #minX, maxX, minY, maxY
myCalibrator.setBasePosition(750,600)
myCalibrator.setPollingTime(0.005) #optimum: 0.005
myCalibrator.setAircraftID(5)

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
             
plt.figure(1)
plt.ion()
plt.axis([-300,300,0,10000])
plt.title('Roll PID error')

plt.figure(2)
plt.ion()
plt.axis([-300,300,0,10000])
plt.title('Pitch PID error')

""" End of debug log initial messages """

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
logger.info("ProgramEnded")
raise SystemExit()
