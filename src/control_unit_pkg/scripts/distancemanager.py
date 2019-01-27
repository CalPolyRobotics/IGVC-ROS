#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt16MultiArray, UInt8, UInt16
#---------------------------------------------------------------------------
#                                Milestone 1
#---------------------------------------------------------------------------
# This piece of code detects people/objects in the front of the LIDAR  and
# changes between Forward, Neutral, Reverse depending on the distance between
# the LIDAR and person/object.
#---------------------------------------------------------------------------

# INITIALIZE GLOBALS

# Half the Width of the Golf Cart
HALF_WIDTH = .546

initFNR = True          # Initialize FNR to forward # TODO: remove
ourDistance = 0         # Holds mimimum distance value from findOurDistance callback
ourVelocity = 0         # Holds velocity from findOurVelocity callback
FNR_State = 1           # Should get the FNR State later
ranges = []             # Holds lidar_scan ranges

# Static distance and velocity threshold variables
x0 = 1.0                # Inner Threshold distance (meters)
x1 = 1.5                # Deadzone Threshold distance (meters)
x2 = 2.0                # Outer Threshold distance (meters)
minVelocity = 50        # minimum velocity in  cm/s
#--------------------------------------------------------------------------


def range_Rectangle(index, length):
    ''' range_Rectangle checks to see if lidar value is in front of golf cart. '''
    #TODO: Change values to laser_scan dependent values
    if index > 541 / 2:
        if length > (HALF_WIDTH / math.cos(math.radians(180 - ((index - 90) / 2)))):
            return False
    else:
        if length > (HALF_WIDTH / math.cos(math.radians((index - 90) / 2))):
            return False
    return True

#---------------------------------------------------------------------------
# Callback to update the minimumDistance to the smallest distance from lidar range
#---------------------------------------------------------------------------
def findOurDistance(data):
    ''' Callback to update the minimumDistance to the smallest distance from lidar range. '''
    global ranges
    global ourDistance
    ranges = data.data
    minimumDistance = 20.0                       # Maximum range of lidar
    # Range of lidar data we want to read (values in front of lidar)
    for i in range(130, 421):
        if range_Rectangle(i, ranges[i]):
            if (ranges[i] < minimumDistance) and (ranges[i] > 0.1):
                # .1 is threshold distance (distance >20m is given as value close to 0)
                minimumDistance = ranges[i]      # Update minimumDistance value
            else:
                pass
    ourDistance = minimumDistance


def findOurVelocity(data):
    ''' Callback to update ourVelocity value. '''
    global ourVelocity
    try:
        ourVelocity = (data.data[0]) / 10
    except:
        pass


def callbackFNR(data):
    ''' Callback to update FNR state '''
    global FNR_state
    FNR_state = data
    print ("Current FNR State is: %s",  data)

#--------------------------------------------------------------------------
# Main State Machine for F,N,R state transitions
#--------------------------------------------------------------------------
def stateMachine():
    ''' Main State Machine for F,N,R state transitions. '''
    global ourVelocity
    global FNR_State
    global ourDistance
    #global initFNR
    temp = FNR_State        # To compare initial to final FNR_State

    # State Machine Logic
    if FNR_State == 1:      # Forward State
        if ((ourDistance < x0)):
            print("We are changing to state 2 (Reverse).")
            print("Our distance to change was: ", ourDistance)
            FNR_State = 2
    elif FNR_State == 0:    # Neutral State
        if (ourDistance > x2):
            print("We are changing to state 1 (Forward).")
            print("Our distance to change was: ", ourDistance)
            FNR_State = 1
        elif (ourDistance < x0):
            print("we are changing to state 0 (Neutral).")
            print("Our distance to change was: ", ourDistance)
            FNR_State = 2
    elif FNR_State == 2:    # Reverse State
        if (ourDistance > x1):
            print("we are changing to state 0 (Neutral).")
            print("Our distance to change was: ", ourDistance)
            FNR_State = 0

    # Publish FNR State if it changed
    if FNR_State != temp:
        publishState(FNR_State)


def publishState(new_state):
   ''' Publishes new state.'''
   PUB_FNR.publish(new_state)
   FNR_State = new_state


def listener():
    ''' Subscribers.'''
    print("Hello! ~ Milestone 1")
    rospy.Subscriber("lidar_scan_ranges", Float32MultiArray, findOurDistance)
    rospy.Subscriber("Get_Speed", UInt16MultiArray, findOurVelocity)
    rospy.Subscriber("Get_FNR", UInt8, callbackFNR)
    while not rospy.is_shutdown():
        stateMachine()  # TODO: Rename StateMachine
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


#---------------------------------------------------------------------------
if __name__ == '__main__':
    ''' Start point. '''
    # Initialize publisher and node
    PUB_FNR = rospy.Publisher('Set_FNR', UInt8, queue_size=10)
    rospy.init_node('distance_manager', anonymous=True)
    # Change initial FNR state to Forward
    print("Initialzation: Changing FNR state to 1.")
    PUB_FNR.publish(1)  # Forward State
    print("Initialzation: Changed FNR state to 1.")
    # Go to main function of node
    listener()
