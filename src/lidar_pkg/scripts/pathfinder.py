#!/usr/bin/env python
import rospy, time, math
from std_msgs.msg import UInt16, Float32MultiArray
from willitcrash import getCrashDistancesCartesian
from numpy import interp
from collision_gui_config import *
#Global Variables
lidarDataArray = []
ANGLE_RANGE = (-25, 25)
COLLISION_THRESHOLD = 1
rangeConsidered = 1
#Variable Initialization
for i in range(0, 540):
    lidarDataArray.append(1)

def WillAngleCrash(steeringAngle):
    collisionsAtAngle = CountLidarCollisions(steeringAngle)
    if (collisionsAtAngle < COLLISION_THRESHOLD): #WilL Crash
        print("Path good; only " + str(collisionsAtAngle) + " collisions")
        return True
    else:
        print("Path bad; there are " + str(collisionsAtAngle) + " collisions")
        return False

def CountLidarCollisions(steeringAngle):
    #print(lidarDataArray)
    collisions = 0
    steerInvert = 1
    if steeringAngle == 0:
        print("OH NO FALSE TARGET RADIANS IS 0!!!")
        collisions = 0
    else:
        if steeringAngle < 0:
            steerInvert = -1
        if steeringAngle > 0:
            steerInvert = 1
    targetRadians = math.radians(steeringAngle)
    circleInnerR = abs(CART_HEIGHT/(math.tan(abs(targetRadians))) - 
                    abs(0.5*CART_HEIGHT/math.tan(abs(targetRadians))))
    circleOuterR = abs(CART_HEIGHT/math.sin(abs(targetRadians)) - 
                    abs(0.5*CART_HEIGHT/math.sin(abs(targetRadians)))) + 2*CART_WIDTH
    circleCenterX = steerInvert*(circleInnerR+CART_WIDTH)
    circleCenterY = CART_HEIGHT

    crashDistances = getCrashDistancesCartesian(circleCenterX, circleCenterY, circleInnerR, circleOuterR)
    for i in range(0, 540):
        crashData = crashDistances[i]
        theta = (i-90)*90/180
        if steeringAngle != 0:
            if (
                ((crashData[0] == -1) and (lidarDataArray[i] < crashData[1])) or
                ((crashData[0] != -1) and ((lidarDataArray[i] < crashData[0]) or 
                ((lidarDataArray[i] > crashData[2]) and (lidarDataArray[i] < crashData[1]))))
                ) and lidarDataArray[i] < rangeConsidered:
                collisions+=1
        elif(steeringAngle == 0):
            if (i > 90 and i < 450):
                if (abs(math.cos(math.radians(abs(theta)))*lidarDataArray[i])<CART_WIDTH):
                    collisionsDetected+=1
        else:
            pass
    print ("Collsions: " + str(collisions))
    return collisions
def AllowedAngles():
    allowedAngles = [] #Initialize empty array of allowedAngles
    #print(allowedAngles)
    #for i in range (ANGLE_RANGE):
    for i in range (1, 25): #ANGLE_RANGE):
        if (WillAngleCrash(i) == True):
            allowedAngles.append(i)
    print("Number of Solutions: " + str(len(allowedAngles)))
    print("Solutions: " + str(allowedAngles))
    return allowedAngles 

def PickSolution(allowedAngles):
    #GPS Logic
    #Road Rules Logic
    #Temporay Logic (Where we JUST PICK ONE!!!)
    solution = interp(allowedAngles[0], [-25, 25], [0,65535])
    return int(solution)

def CallbackLidar(data):
    global lidarDataArray
    for i in range(0, 540):
        lidarDataArray[i] = data.data[i]

def Pathfinder():
    rospy.init_node('Pathfinder', anonymous=True)
    pubSolution = rospy.Publisher("Set_Solution", UInt16, queue_size=1000)
    rospy.Subscriber("lidar_scan_ranges", Float32MultiArray, CallbackLidar)
    while not rospy.is_shutdown():
        myBoolean = WillAngleCrash(7)
        print(myBoolean)
        allowedAngles = AllowedAngles()
        print(allowedAngles)
        try:
            finalSolution = PickSolution(allowedAngles)
            print(finalSolution)
            pubSolution.publish(finalSolution)
        except:
            print("No Solution RIP")
        time.sleep(2) #Temporary

if __name__ == "__main__":
    print("Pathfinder Running")
    Pathfinder()
