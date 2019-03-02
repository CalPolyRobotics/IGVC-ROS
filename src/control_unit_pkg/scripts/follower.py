#!/usr/bin/env python
import rospy, math, time
#from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt16MultiArray, UInt8, UInt16, Int16
# Declare Publishers

#Global
currentVelocity = 0

#Static
target = 4.0
beta = 0 #Increase this to increase faster
kappa = 0.5
totalError = 0
maxSpeed = 6.7 #Max speed of golf cart at full throttle (m/s), empirical
translationFactor = float(63/maxSpeed) #0-63 (2^6 -1) DAC has 6 bit resolution

pubThrottle = rospy.Publisher("Set_Throttle", UInt16, queue_size=1)

def getVelocity(data):
    #print("I ran getVelocity")
    global currentVelocity
    try:
        #ourVelocity = (data.data[0])/10
        currentVelocity = data.data / 1000.0
    except:
        #print("I died")
        pass
    print(currentVelocity)

def cruiseControl(target):
    global totalError
    error = currentVelocity - target
    totalError = totalError + error
    print("Error: " , error)
    print("TotalError: " , totalError)
    temp =  int (target*translationFactor - error*beta - totalError*kappa)
    if temp > 63:
        pubThrottle.publish(63)
    elif temp < 0:
        pubThrottle.publish(0)
    else:
        pubThrottle.publish(temp)



def listener():
    print("Hello")
    rospy.Subscriber("Get_Speed", Int16, getVelocity)
    rospy.init_node('Cruise_Control', anonymous=True)
    while not rospy.is_shutdown():
        #print("I looped")
        time.sleep(0.35)
        global target
        cruiseControl(target)
    rospy.spin()

if __name__ == '__main__':
#rospy.init_node('Cruise_Control', anonymous=True)
    listener()

