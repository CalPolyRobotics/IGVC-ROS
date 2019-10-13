#!/usr/bin/env
import threading
import time
import rospy
import math
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt8
from std_msgs.msg import UInt16


class Vehicle(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

        # Setup ROS node and publishers`
        rospy.init_node('golf_cart', anonymous=True)

        PUB_FNR = rospy.Publisher('Set_FNR', UInt8, queue_size=10)
        PUB_THROTTLE = rospy.Publisher('Set_Throttle', UInt16, queue_size=10)
        PUB_STEERING = rospy.Publisher('Set_Steering', UInt16, queue_size=10)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    # Gets the current FNR state
    def getFNR(self):
        # FNR_current = @@@ ROS COMMAND TO GET FNR STATE GOES HERE @@@
        return FNR_current

    # Sets the FNR state
    def setFNR(self, FNR_desired):
        # Enumerate the different gearshift states
        NEUTRAL = 0
        FORWARD = 1
        REVERSE = 2

        returnval = -1

        FNR_current = self.getFNR()

        # Only do something if the desired is valid and different from the current and the 
        # throttle is zero
        if (((FNR_desired == NEUTRAL) or (FNR_desired == FORWARD) or
             (FNR_desired == REVERSE)) and (FNR_desired != FNR_current)):
            PUB_FNR.publish(FNR_desired)
            returnval = 0

        return returnval

    # Returns current steering value
    def getSteering(self):
        return

    # Sets steering to specific angle
    def setSteering(self, value):
        MAX_VALUE = 65535
        MIN_VALUE = 0

        returnval = -1
        value_desired_int = int(value_desired)
        value_current = self.getThrottle()

        if ((value_desired_int >= MIN_VALUE) and
            (value_desired_int <= MAX_VALUE) and
            (value_desired_int != value_current)):
            PUB_STEERING.publish(value_desired_int)
            returnval = 0

        return returnval

    # Gets current throttle amount
    def getThrottle(self):
        return

    # Sets throttle amount
    def setThrottle(self, throttle_desired):
        MAX_VALUE = 100
        MIN_VALUE = 0

        returnval = -1
        throttle_desired_int = int(throttle_desired)
        throttle_current = self.getThrottle()

        if ((throttle_desired_int >= MIN_VALUE) and
            (throttle_desired_int <= MAX_VALUE) and
            (throttle_desired_int != throttle_current)):
            PUB_THR.publish(throttle_desired_int)
            returnval = 0

        return returnval

    # Gets current brake amount
    def getBrakes(self):
        return

    # Sets brake amount
    def setBrakes(self, value):
        MAX_VALUE = 100
        MIN_VALUE = 0

        returnval = -1
        value_desired_int = int(value_desired)
        value_current = self.getBrakes()

        if ((value_desired_int >= MIN_VALUE) and
            (value_desired_int <= MAX_VALUE) and
            (value_desired_int != value_current)):
            PUB_BRAKES.publish(value_desired_int)
            returnval = 0

        return returnval


if __name__ == '__main__':
    myVehicle = Vehicle()
