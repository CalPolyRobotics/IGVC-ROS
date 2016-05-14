#!/usr/bin/env python

import rospy, pygame, sys, math
from std_msgs.msg import UInt8, UInt16, Float32MultiArray
from willitcrash import getCrashDistancesCartesian
from numpy import interp

#Global Variables
lidarDataArray = []
publishedSteering = 0
publishedFNR = 0
publishedThrottle = 0

#Variable Initialization
for i in range (0, 540):
    lidarDataArray.append(1)
publishedSteering = 5

#GUI_CONSTANTS
SCALE_FACTOR = 27
SCREEN_DIMENSIONS = WIDTH, HEIGHT = 1375, 825
SCREEN_LABEL = "IGVC HUD"
BACKGROUND_COLOR = (139,131,134)
ORIGIN = X_AXIS_ORIGIN, Y_AXIS_ORIGIN = 20,20
AXIS_COLOR = (0,0,0)
AXIS_LENGTH = 20
MARKER_COLOR = (150,150,150)
TEXT_COLOR = (50,50,50)
#GOLF_CART_CONSTANTS
GOLF_CART_COLOR = (74,74,74)
CART_WIDTH = 45*2.54/2/100
CART_HEIGHT = 65*2.54/100

#Subscription Callbacks
def CallbackLidar(data):
    global lidarDataArray
    for i in range(0, 540):
        lidarDataArray[i] = data.data[i]

def CallbackSteering(data):
    print("callbackSteering run")
    global publishedSteering
    publishedSteering = data

def CallbackFNR(data):
    global publishedFNR
    publishedFNR = data

def CallbackThrottle(data):
    global publishedThrottle
    publishedThrottle = data

def InitializeGUI():
    global screen
    global font
    pygame.init()
    screen = pygame.display.set_mode(SCREEN_DIMENSIONS)
    pygame.display.set_caption(SCREEN_LABEL)
    screen.fill(BACKGROUND_COLOR)
    font = pygame.font.SysFont('Arial', 20)
    pygame.display.update()

def DrawLine(color, startPoint, endPoint):
    pygame.draw.line(screen, color, startPoint, endPoint)

def DrawRectangle(color, startPoint, width, height):
    pygame.draw.rect(screen, color, (startPoint, (width, height)), 0)

def DrawCircle(color, xCoord, yCoord, radius, size):
    pygame.draw.circle(screen, color, (xCoord, yCoord), radius, size)

def DrawStaticObjects():
    #Draws Coordinate System
    DrawLine(AXIS_COLOR,
            (SCALE_FACTOR*X_AXIS_ORIGIN, SCALE_FACTOR*(Y_AXIS_ORIGIN+AXIS_LENGTH)),
            (SCALE_FACTOR*X_AXIS_ORIGIN, SCALE_FACTOR*(Y_AXIS_ORIGIN-AXIS_LENGTH)))
    DrawLine(AXIS_COLOR,
            (SCALE_FACTOR*(X_AXIS_ORIGIN-AXIS_LENGTH), SCALE_FACTOR*Y_AXIS_ORIGIN),
            (SCALE_FACTOR*(X_AXIS_ORIGIN+AXIS_LENGTH), SCALE_FACTOR*Y_AXIS_ORIGIN))
    #Draws Golf Cart
    DrawRectangle(GOLF_CART_COLOR,
            (SCALE_FACTOR*(X_AXIS_ORIGIN-CART_WIDTH), (SCALE_FACTOR*Y_AXIS_ORIGIN)),
            SCALE_FACTOR*(2*CART_WIDTH), (SCALE_FACTOR*CART_HEIGHT))
    #Draws Measurement Markers
    for i in range (1, 5):
        DrawCircle(MARKER_COLOR, SCALE_FACTOR*X_AXIS_ORIGIN, SCALE_FACTOR*Y_AXIS_ORIGIN, i*SCALE_FACTOR*5, 1)

#Supports 9 Printouts
def DebugPrint(title, text, index):
    titleLength = len(str(title))
    screen.blit(font.render(str(title), True, TEXT_COLOR), (-10+WIDTH-10*titleLength, HEIGHT-45*(2*index+1)))

    textLength= len(str(text))
    screen.blit(font.render(str(text), True, TEXT_COLOR), (-10+WIDTH-10*textLength, -30+HEIGHT-45*(2*index+1)))

def DrawAllText():
    DebugPrint("FNR State", publishedFNR, 0)
    DebugPrint("Throttle", publishedThrottle, 1)
    DebugPrint("Steering", publishedSteering, 2)
    #Max DebugPrintIndex is 9
    for i in range (4, 9):
        DebugPrint("hellohello", "worldworldworld", i)

def RefreshGUI():
    screen.fill(BACKGROUND_COLOR)
    DrawStaticObjects()
    DrawAllText()
    pygame.display.update()

def DrawLidarData():
    crashDistances = getCrashDistancesCartesian(pathParams[0], pathParams[1], pathParams[2], pathParams[3])

def DrawTargetPath():
    pass

def DrawActualPath():
    pass


def TurnGUI():
    rospy.init_node('TurnGUI', anonymous=True)
    #Publisher Initialization
    pubLights = rospy.Publisher("Set_Lights", UInt16, queue_size=1000)
    #Subscriber Initialization
    rospy.Subscriber("lidar_scan_ranges", Float32MultiArray, CallbackLidar)
    rospy.Subscriber("Set_Steering", UInt16, CallbackSteering)
    rospy.Subscriber("Set_FNR", UInt8, CallbackFNR)
    rospy.Subscriber("Set_Throttle", UInt16, CallbackThrottle)
    InitializeGUI()
    while not rospy.is_shutdown():
        print("publishedSteering: " + str(publishedSteering))
        print("publishedFNR: " + str(publishedFNR))
        print("publishedThrottle: " + str(publishedThrottle))
        RefreshGUI()

if __name__ == "__main__":
    print("TurnGUI Running")
    TurnGUI()
