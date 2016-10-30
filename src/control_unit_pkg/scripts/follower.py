#!/usr/bin/env python
import rospy, math
#from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt16MultiArray, UInt8, UInt16
# Declare Publishers

#Global
currentVelocity = 0

#Static
target = 1.0
beta = 1.0
translationFactor = 1.0

def getVelocity(data):
    global ourVelocity
    try:
        #ourVelocity = (data.data[0])/10
        ourVelocity = (data.data[0])/1000
    except:
        pass
    #print(ourVelocity)

def cruiseControl(target):
    error = currentVelocity - target
    pubThrottle.publish(target*translationFactor- error*beta)

def listener():
    print("Hello")
    rospy.Subscriber("Get_Speed", UInt16MultiArray, getVelocity)
    pubThrottle = rospy.Publisher("Set_Throttle", UInt16, queue_size=1)
    rospy.init_node('Cruise_cControl', anonymous=True)
    while not rospy.is_shutdown():
        global target
        cruiseControl(target)

if __name__ == '__main__':
    listener()
