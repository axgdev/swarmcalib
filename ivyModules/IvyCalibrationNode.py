#!/usr/bin/python2
import rospy
from ivy.std_api import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import time
import kill_log

class IvyCalibrationNode:
    def IvyInitStart(self):
        """ Initializes the Ivy Server and ROS Subscriber

        Should only be called once per session.
        """
        try:
            IvyInit('Calibration Node', '', 0)
        except AssertionError:
            print('Assertion Error in IvyInit(!= none), is there a server already running? Exiting')
            IvyStop()
            raise SystemExit()
        IvyStart()
        try:
            self.initRosSub()
        except rospy.exceptions.ROSInitException as e:
            print('\nInitialization failed due to ROS error, exiting...')
            self.IvyInitStop()
        time.sleep(1)
        print('Ivy Calibration Node ready!')
        self.myKillLog = kill_log.KillLog()
        self.myKillLog.startLog()


    def IvyInitStop(self):
        """Stops the Ivy Server.
        """
        time.sleep(1)
        IvyStop()
        self.myKillLog.stopLog()


    def handlePos(self, data):
        """ Callback for the ROS subscriber.

        """
        global copterPos
        copterPos=data
        self.myKillLog.setPositionThreaded([copterPos.x,copterPos.y,copterPos.theta])


    def initRosSub(self):
        """ Initializes the ROS subscriber.

        Is automatically called during the Ivy initialization process
        in IvyInitStart().
        """
        try:
            rospy.init_node('poseListener', anonymous=False)
        except KeyboardInterrupt:
            print('\nROS initialization canceled by user')
        except rospy.exceptions.ROSInitException as e:
            print('\nError Initializing ROS:')
            print(str(e))
            raise

        rospy.Subscriber("/copter", Pose2D, self.handlePos)


    def IvyGetPos(self):
        """Simply returns the position grabbed via ROS to the caller

        """
        try:
            return copterPos
        except NameError as e:
            print("CopterPos not yed defined! (No data from ROS?):")
            print(str(e))

    def IvyGetPosList(self):
        """Returns the position to a list for less dependency with ros
           Returns
           -------
           List
        """
        position = self.IvyGetPos()
        return [position.x, position.y, position.theta]

    def IvySendCalib(self,param_ID, AC_ID, value):
        """Sends the given parameter via Ivy

        AC_ID:      ID of the aricraft to be calibrated
        param_ID:   ID of the parameter to be calibrated
                     phi   = 58 roll
                     theta = 59 pitch
                     psi   = 60 yaw
        value:      value to be set for the parameter !in degrees!
        """
        print("sending calib msg")
        IvySendMsg('dl SETTING %d %d %f' %
                    (AC_ID,
                    param_ID,
                    value
                    ))


    def IvySendKill(self, AC_ID):
        """Sends a kill message to the aircraft

        """
        IvySendMsg('dl KILL %d 1' %
                    (AC_ID
                    ))
        self.myKillLog.killSignalReceived = True

    def IvySendCalParams(self, AC_ID, turn_leds, roll, pitch, yaw):
        IvySendMsg('dl CALPARAMS %d %d %f %f %f' %
                    (AC_ID, turn_leds, roll, pitch, yaw
                    ))


    def IvySendUnKill(self, AC_ID):
        """Sends an unkill message to the aircraft

        """
        IvySendMsg('dl KILL %d 0' %
                    (AC_ID
                    ))



    def IvySendSwitchBlock(self, AC_ID, block_ID):
        """Sends a message to switch the flight plan

        """
        IvySendMsg('dl BLOCK %d %d' %
                    (block_ID,
                     AC_ID,
                     ))
