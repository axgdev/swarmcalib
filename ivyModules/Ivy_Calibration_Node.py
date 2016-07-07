#!/usr/bin/python2
import rospy
from ivy.std_api import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import time

def IvyInitStart():
    """ Initializes the Ivy Server and ROS Subscriber

    Should only be called once per session.
    """
    IvyInit('Calibration Node', '', 0)
    IvyStart()
    initRosSub()
    time.sleep(1)
    print('Ivy Calibration Node ready!')

def IvyGetPos():
    """Simply returns the position grabbed via ROS to the caller

    """
    return copterPos

def IvySendCalib(AC_ID, param_ID, value):
    """Sends the given parameter via Ivy

    param_IDs:  phi   = 58
                theta = 59
                psi   = 60
    """

    IvySendMsg('dl DL_SETTING %d %d %f' %
                (AC_ID,
                param_ID,
                value
                ))

def IvyInitStop():
    """Stops the Ivy Server.

    """
    time.sleep(5)
    IvyStop()

def handlePos(data):
    """ Callback for the ROS subscriber.


    """
    global copterPos
    copterPos=data


def initRosSub():
    """ Initializes the ROS subscriber.

    Is automatically called during the Ivy initialization process
    in IvyInitStart().
    """
    rospy.init_node('poseListener', anonymous=False)
    rospy.Subscriber("copters/0/pose", Pose2D, handlePos)

def IvyGetParam():
    IvySendMsg('dl GET_DL_SETTING 5 58')
