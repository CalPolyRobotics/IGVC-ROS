#!/usr/bin/env python
import rospy, math
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt16MultiArray, UInt8, UInt16
# Half the Width of the Golf Cart
HALF_WIDTH = .546
# Declare Publishers


#Changing
initFNR = True
ourDistance = 0
ourVelocity= 0
#Should get the FNR State later
#FNR_State = 1
FNR_State = 1
ranges = []
#static
x0 = 2.0
x1 = 5.0
minVelocity = 50 #cm/s
def range_Rectangle(index, length):
    #TODO: Change values to laser_scan dependent values
    if index > 541/2:
        if length > (HALF_WIDTH/math.cos(math.radians(180-((index - 90)/2)))):
            return False
    else:
        if length > (HALF_WIDTH/math.cos(math.radians((index - 90)/2))):
            return False
    return True

def findOurDistance(data):
    global ranges
    global ourDistance
    ranges = data.data;
    minimumDistance = 20.0
    for i in range(130, 421): # Range of lidar data we want to reed
        if range_Rectangle(i, ranges[i]):
            #print(ranges[i])
            if (ranges[i] < minimumDistance) and (ranges[i] > 0.1):
                minimumDistance = ranges[i]
            else:
                #print("Something was in the rectangle but far away")
                pass
            #print "In Range Rectangle"
    ourDistance = minimumDistance
    #print(minimumDistance)
    #print("The distance calculated to nearest: " , ourDistance)

#Is the cart moving? How fast?
def findOurVelocity(data):
    global ourVelocity
    try:
        ourVelocity = (data.data[0])/10
    except:
        pass
        #print("Error: data.data broke")
        #print(data.data)
    #print(ourVelocity)
    #print"SPEED is : ", ourVelocity

def stateMachine():
    global ourVelocity
    #print("Our velocity: " , ourVelocity)
    global FNR_State
    #print("Our FNR State: " , FNR_State)
    global ourDistance
    #print("Our Distance: " , ourDistance)
    #global initFNR
    temp = FNR_State
    #State Machine Logic
    if FNR_State == 1:
        #print("we are in state 1")
        if ((ourDistance < x0) and (ourVelocity < minVelocity)):
            print("We are changing to state 1.")
            print("Our distance to change was: " , ourDistance)
            FNR_State = 0
        elif ((ourDistance < x0) and (ourVelocity >= minVelocity)):
            print("We are changing to state 2.")
            print("Our distance to change was: " , ourDistance)
            FNR_State = 2
    elif FNR_State == 0:
        #print("We are in state 0.")
        if (ourDistance > x1):
            print("We are changing to state 1.");
            print("Our distance to change was: " , ourDistance)
            FNR_State = 1
    elif FNR_State == 2:
        #print("We are in state 2.")
        if (ourDistance > x0):
            print("we are changing to state 0")
            print("Our distance to change was: " , ourDistance)
            FNR_State = 0

    #Publish FNR State if it changed
    if FNR_State != temp:
        publishState(FNR_State)
    """
    if initFNR == True:
        initFNR = False
        FNR_State = 1
        publishState(FNR_State)
    """

def publishState(new_state):
   PUB_FNR.publish(new_state)
   FNR_State = new_state

def listener():
    print("Hello")
    #rospy.init_node('proximity_manager', anonymous=True)
    rospy.Subscriber("lidar_scan_ranges", Float32MultiArray, findOurDistance)
    rospy.Subscriber("Get_Speed", UInt16MultiArray, findOurVelocity)
    rospy.Subscriber("Get_FNR", UInt8, callbackFNR)
    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        stateMachine() #TODO: Rename StateMachine
    rospy.spin()

def callbackFNR(data):
    global FNR_state
    FNR_state = data
    print ("Current State is: %s",  data)

if __name__ == '__main__':
    PUB_FNR = rospy.Publisher('Set_FNR', UInt8, queue_size=10)
    rospy.init_node('distance_manager', anonymous=True)
    print("Initialzation: Changing FNR state to 1.")
    PUB_FNR.publish(1)
    print("Initialzation: Changed FNR state to 1.")

    #PUB_FNR.publish(1)
    listener()
