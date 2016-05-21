#!/usr/bin/env python

import rospy, pygame, sys, math, os, time
from std_msgs.msg import UInt8, UInt16, Float32MultiArray
from willitcrash import getCrashDistancesCartesian
from numpy import interp

#Global Variables
virtualMode = 0
democracy = 1
lidarDataArray = []
publishedSteering = 0
publishedFNR = 0
publishedThrottle = 0
publishedCruiseControl = 0
#publishedSteerControl = 0
actualSteering = 32766
actualAngle = 0

#Variable Initialization
for i in range (0, 540):
    lidarDataArray.append(1)
publishedSteering = 32767
collisionsDetected = 0

#GUI_BACKGROUNDS
BACKGROUND_IMAGE = pygame.image.load(os.path.abspath("src/lidar_pkg/scripts/images/Background.bmp"))
LOGO_IMAGE = pygame.image.load(os.path.abspath("src/lidar_pkg/scripts/images/CPRCLogo.bmp"))

#GUI_CONSTANTS
SCALE_FACTOR = 30
SCREEN_DIMENSIONS = WIDTH, HEIGHT = 1375, 850
SCREEN_LABEL = "IGVC HUD"
BACKGROUND_COLOR = (73,87,83)

ORIGIN = X_AXIS_ORIGIN, Y_AXIS_ORIGIN = 20,20
AXIS_COLOR = (0,0,0)
AXIS_LENGTH = 20
MARKER_COLOR = (100,100,100)
TEXT_COLOR = (171,173,144)
#LIDAR & STEERING CSONTANTS
CIRCLE_COLOR = (175,175,175)
TRUE_CIRCLE_COLOR = (255,255,255)
#GOLF_CART_CONSTANTS
#GOLF_CART_COLOR = (74,74,74)
GOLF_CART_COLOR = (25,25,25)
#GOLF_CART_COLOR = (86,82,72)
CART_WIDTH = 45*2.54/2/100
CART_HEIGHT = 65*2.54/100
MAX_STEERING_ANGLE = 25
MAX_STEERING_VALUE = 65535
HALF_MAX_STEERING_VALUE = 32767

#Don't Worry About This
if democracy == 1:
    BACKGROUND_IMAGE = pygame.image.load(os.path.abspath("src/lidar_pkg/scripts/images/Space2.bmp"))
    #LOGO_IMAGE = pygame.image.load(os.path.abspath("src/lidar_pkg/scripts/images/GUI_Pinto_Horse.bmp"))
    LOGO_IMAGE = pygame.image.load(os.path.abspath("src/lidar_pkg/scripts/images/CPRCLogo.bmp"))
    GOLF_CART_COLOR = (255,153,0)
    AXIS_COLOR = (185,185,185)
    TEXT_COLOR = (255,255,255)

#Subscription Callbacks
def CallbackLidar(data):
    global lidarDataArray
    for i in range(0, 540):
        lidarDataArray[i] = data.data[i]

def CallbackSteering(data):
    #print("callbackSteering run")
    global publishedSteering
    publishedSteering = data.data

def CallbackGetSteerValue(data):
    global actualSteering
    actualSteering = -1*interp(int(data.data), [575,1570], [-32767, 32767])

def CallbackFNR(data):
    global publishedFNR
    publishedFNR = data.data

def CallbackThrottle(data):
    global publishedThrottle
    publishedThrottle = data.data

def CallbackCruiseControl(data):
    global publishedCruiseControl
    publishedCruiseControl = data.data

def InitializeGUI():
    global screen
    global font
    pygame.init()
    screen = pygame.display.set_mode(SCREEN_DIMENSIONS)
    pygame.display.set_caption(SCREEN_LABEL)
    screen.fill(BACKGROUND_COLOR)
    #font = pygame.font.SysFont('Arial', 20)
    #font = pygame.font.SysFont('freesans', 20)
    #font = pygame.font.SysFont('cmmi10', 20)
    font = pygame.font.SysFont('dejavusansmono', 15)


    print(pygame.font.get_fonts())
    pygame.display.update()

def DrawLine(color, startPoint, endPoint):
    pygame.draw.line(screen, color, startPoint, endPoint)

def DrawRectangle(color, startPoint, width, height, lineWidth):
    pygame.draw.rect(screen, color, (startPoint, (width, height)), lineWidth)

def DrawCircle(color, xCoord, yCoord, radius, size):
    pygame.draw.circle(screen, color, (int(xCoord), int(yCoord)), int(radius), size)

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
            SCALE_FACTOR*(2*CART_WIDTH), (SCALE_FACTOR*CART_HEIGHT), 0)
    #Draws Measurement Markers
    for i in range (1, 5):
        DrawCircle(AXIS_COLOR, SCALE_FACTOR*X_AXIS_ORIGIN, SCALE_FACTOR*Y_AXIS_ORIGIN, i*SCALE_FACTOR*5, 1)

#Supports 9 Printouts
def DebugPrint(title, text, index):
    titleLength = len(str(title))
    screen.blit(font.render(str(title), True, TEXT_COLOR), (-10+WIDTH-10*titleLength, -30+HEIGHT-35*(2*index+1)))

    textLength= len(str(text))
    screen.blit(font.render(str(text), True, TEXT_COLOR), (-10+WIDTH-10*textLength, HEIGHT-35*(2*index+1)))

def DrawAllText():
    global collisionsDetected
    DebugPrint("FNR State", publishedFNR, 0)
    DebugPrint("Throttle", publishedThrottle, 1)
    DebugPrint("Steering", publishedSteering, 2)
    DebugPrint("Collisions", collisionsDetected, 7)
    DebugPrint("CruiseControl", publishedCruiseControl, 5)
    DebugPrint("GetSteerValue", actualSteering, 4)

    collisionsDetected = 0
    #Max DebugPrintIndex is 8

def RefreshGUI():
    #screen.fill(BACKGROUND_COLOR)
    screen.blit(BACKGROUND_IMAGE, (0,0))
    screen.blit(LOGO_IMAGE, (WIDTH-237,0))
    DrawStaticObjects()
    DrawAllText()

def returnPathParams():
    targetAngle = float(actualSteering-HALF_MAX_STEERING_VALUE)
    if virtualMode == 1:
        targetAngle = float(publishedSteering -HALF_MAX_STEERING_VALUE)
    targetRadians = math.radians((targetAngle*25/HALF_MAX_STEERING_VALUE))
    steerInvert = 1
    circleCenterX = 1
    circleCenterY = 1
    circleInnerR = 1
    circleOuterR = 1

    if targetRadians == 0:
        print("OH NO FALSE TARGET RADIANS IS 0!!!")
    else:
        if targetRadians < 0:
            steerInvert = -1
        if targetRadians > 0:
            steerInvert = 1

        circleInnerR = abs(CART_HEIGHT/(math.tan(abs(targetRadians))) - 
                abs(0.5*CART_HEIGHT/math.tan(abs(targetRadians))))
        circleOuterR = abs(CART_HEIGHT/math.sin(abs(targetRadians)) - 
                abs(0.5*CART_HEIGHT/math.sin(abs(targetRadians)))) + 2*CART_WIDTH
        circleCenterX = steerInvert*(circleInnerR+CART_WIDTH)
        circleCenterY = CART_HEIGHT
    return (circleCenterX, circleCenterY, circleInnerR, circleOuterR)

def DrawLidarData():
    pathParams = returnPathParams()
    #DebugPrint("pathParams[0]", pathParams[0], 3)
    #DebugPrint("pathParams[1]", pathParams[1], 4)
    #DebugPrint("pathParams[2]", pathParams[2], 5)
    #DebugPrint("pathParams[3]", pathParams[3], 6)

    crashDistances = getCrashDistancesCartesian(pathParams[0], pathParams[1], pathParams[2], pathParams[3])
    for i in range(0, 540):
        crashData = crashDistances[i]
        theta = (i-90)*90/180
        pointColor = (204,0,51)
        if lidarDataArray[i] < 0.05:
            pointColor = (0,51,205)
        elif(
            ((crashData[0] == -1) and (lidarDataArray[i] < crashData[1])) or
            ((crashData[0] != -1) and ((lidarDataArray[i] < crashData[0]) or 
            ((lidarDataArray[i] > crashData[2]) and (lidarDataArray[i] < crashData[1]))))
            ):
            pointColor = (204,0,51)
            global collisionsDetected
            collisionsDetected+=1
        else:
            pointColor = (51,204,0)

        #Draws Lidar Points
        cos_function = lidarDataArray[i]*(math.cos(math.radians(theta)))
        sin_function = lidarDataArray[i]*(math.sin(math.radians(theta)))
        DrawCircle(pointColor, SCALE_FACTOR*(X_AXIS_ORIGIN+cos_function), SCALE_FACTOR*(Y_AXIS_ORIGIN-sin_function), 3,3)

        #Draws Collision Circles
        #Draws Crash Distance Outer Circle
        cos_function = crashData[1]*(math.cos(math.radians(theta)))
        sin_function = crashData[1]*(math.sin(math.radians(theta)))
        DrawCircle((0,0,50), SCALE_FACTOR*(X_AXIS_ORIGIN+cos_function), SCALE_FACTOR*(Y_AXIS_ORIGIN-sin_function), 2,2)

        #Draws Crash Distance Inner Circle
        if (crashData[0] != -1):
            cos_function = crashData[0]*(math.cos(math.radians(theta)))
            sin_function = crashData[0]*(math.sin(math.radians(theta)))
            DrawCircle((0,0,50), SCALE_FACTOR*(X_AXIS_ORIGIN+cos_function), SCALE_FACTOR*(Y_AXIS_ORIGIN-sin_function), 2,2)

            cos_function = crashData[2]*(math.cos(math.radians(theta)))
            sin_function = crashData[2]*(math.sin(math.radians(theta)))
            DrawCircle((0,0,50), SCALE_FACTOR*(X_AXIS_ORIGIN+cos_function), SCALE_FACTOR*(Y_AXIS_ORIGIN-sin_function), 2,2)


def DrawTargetPath():
    #Target Steering Angle
    targetAngle = float(publishedSteering-HALF_MAX_STEERING_VALUE)
    targetRadians = (targetAngle*25*math.pi/180/HALF_MAX_STEERING_VALUE)
    #targetRadians = math.radians((targetAngle*25/HALF_MAX_STEERING_VALUE))
    DebugPrint("Angle", (targetRadians)*180/math.pi, 8)
    if targetAngle < 5 and targetAngle > -5:
        DrawRectangle(CIRCLE_COLOR, (SCALE_FACTOR*(Y_AXIS_ORIGIN-CART_WIDTH), 0), SCALE_FACTOR*CART_WIDTH*2, HEIGHT, 2)
    elif targetRadians != 0:
        if targetRadians < 0:
            steerInvert = -1
        if targetRadians > 0:
            steerInvert = 1
        innerRadius = abs(CART_HEIGHT/(math.tan(abs(targetRadians))) - 
                    abs(0.5*CART_HEIGHT/math.tan(abs(targetRadians))))
        outerRadius = abs(CART_HEIGHT/math.sin(abs(targetRadians)) - 
                    abs(0.5*CART_HEIGHT/math.sin(abs(targetRadians)))) + 2*CART_WIDTH
        DebugPrint("innerRadius", innerRadius, 6)

        #Draw Inner Circle
        DrawCircle(CIRCLE_COLOR, 
                SCALE_FACTOR*(steerInvert*innerRadius+X_AXIS_ORIGIN+steerInvert*(CART_WIDTH)), 
                SCALE_FACTOR*(Y_AXIS_ORIGIN+CART_HEIGHT), 
                SCALE_FACTOR*innerRadius, 3)

        #Draw Outer Circle
        DrawCircle(CIRCLE_COLOR, 
                SCALE_FACTOR*(steerInvert*innerRadius+X_AXIS_ORIGIN+steerInvert*(CART_WIDTH)), 
                SCALE_FACTOR*(Y_AXIS_ORIGIN+CART_HEIGHT), 
                SCALE_FACTOR*outerRadius, 3)

def DrawActualPath():
    #Target Steering Angle
    targetAngle = actualSteering-HALF_MAX_STEERING_VALUE
    targetRadians = math.radians((targetAngle*25/HALF_MAX_STEERING_VALUE))
    if targetAngle < 5 and targetAngle > -5:
        DrawRectangle(TRUE_CIRCLE_COLOR, (SCALE_FACTOR*(Y_AXIS_ORIGIN-CART_WIDTH), 0), SCALE_FACTOR*CART_WIDTH*2, HEIGHT, 2)
    elif targetRadians != 0:
        if targetRadians < 0:
            steerInvert = -1
        if targetRadians > 0:
            steerInvert = 1
        innerRadius = abs(CART_HEIGHT/(math.tan(abs(targetRadians))) - 
                    abs(0.5*CART_HEIGHT/math.tan(abs(targetRadians))))
        outerRadius = abs(CART_HEIGHT/math.sin(abs(targetRadians)) - 
                    abs(0.5*CART_HEIGHT/math.sin(abs(targetRadians)))) + 2*CART_WIDTH

        #Draw Inner Circle
        DrawCircle(TRUE_CIRCLE_COLOR, 
                SCALE_FACTOR*(steerInvert*innerRadius+X_AXIS_ORIGIN+steerInvert*(CART_WIDTH)), 
                SCALE_FACTOR*(Y_AXIS_ORIGIN+CART_HEIGHT), 
                SCALE_FACTOR*innerRadius, 3)

        #Draw Outer Circle
        DrawCircle(TRUE_CIRCLE_COLOR, 
                SCALE_FACTOR*(steerInvert*innerRadius+X_AXIS_ORIGIN+steerInvert*(CART_WIDTH)), 
                SCALE_FACTOR*(Y_AXIS_ORIGIN+CART_HEIGHT), 
                SCALE_FACTOR*outerRadius, 3)

def TurnGUI():
    rospy.init_node('TurnGUI', anonymous=True)
    #Publisher Initialization
    pubLights = rospy.Publisher("Set_Lights", UInt16, queue_size=1000)
    #Subscriber Initialization
    rospy.Subscriber("lidar_scan_ranges", Float32MultiArray, CallbackLidar)
    rospy.Subscriber("Set_Steering", UInt16, CallbackSteering)
    rospy.Subscriber("Set_FNR", UInt8, CallbackFNR)
    rospy.Subscriber("Set_Throttle", UInt16, CallbackThrottle)
    rospy.Subscriber("Set_CruiseControl", UInt8, CallbackCruiseControl)
    rospy.Subscriber("GetSteerValue", UInt8, CallbackGetSteerValue)
    InitializeGUI()
    while not rospy.is_shutdown():
        RefreshGUI()
        DrawTargetPath()
        DrawActualPath()
        DrawLidarData()
        pygame.display.update()

if __name__ == "__main__":
    print("TurnGUI Running")
    TurnGUI()
