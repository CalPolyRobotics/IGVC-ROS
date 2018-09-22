#!/usr/bin/env python

import rospy, pygame, sys, math, os, time
import pygame as pyg
from std_msgs.msg import UInt8, UInt16, Float32MultiArray
from willitcrash import getCrashDistancesCartesian
from numpy import interp
from collision_gui_config import *
#from pygame_functions import *

#Global Variables
virtualMode = 1
democracy = 0
lidarDataArray = []
publishedSteering = 0
publishedFNR = 0
publishedThrottle = 0
publishedCruiseControl = 0
actualSteering = 32766
actualAngle = 0
rangeConsidered = 20
fullscreen = True
#SCREEN_DIMENSIONS = WIDTH, HEIGHT = (1000, 500)

#Variable Initialization
for i in range (0, 540):
    lidarDataArray.append(1)
publishedSteering = 32767
collisionsDetected = 0

#GUI_BACKGROUNDS
BACKGROUND_IMAGE = pygame.image.load(os.path.abspath("src/lidar_pkg/scripts/images/Background.bmp"))
LOGO_IMAGE = pygame.image.load(os.path.abspath("src/lidar_pkg/scripts/images/CPRCLogo.bmp"))

#Don't Worry About This
if democracy == 1:
    BACKGROUND_IMAGE = pygame.image.load(os.path.abspath("src/lidar_pkg/scripts/images/Space.bmp"))
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
    #actualSteering = data.data
    actualSteering = -1*interp(int(data.data), [0,65534], [-32767, 32767])

    #actualSteering = -1*interp(int(data.data), [1100,2170], [-32767, 32767])

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
    #screen = pygame.display.set_mode((SCREEN_DIMENSIONS), pyg.RESIZABLE)
    screen = pygame.display.set_mode((0,0), pyg.FULLSCREEN)
    global SCREEN_DIMENSIONS
    global WIDTH
    global HEIGHT
    SCREEN_DIMENSIONS = WIDTH, HEIGHT = screen.get_size()



    pygame.display.set_caption(SCREEN_LABEL)
    screen.fill(BACKGROUND_COLOR)
    #font = pygame.font.SysFont('Arial', 20)
    #font = pygame.font.SysFont('freesans', 20)
    #font = pygame.font.SysFont('cmmi10', 20)
    font = pygame.font.SysFont('dejavusansmono', 15)
    #font = pygame.font.SysFont('loma', 15)

    #print(pygame.font.get_fonts())
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

def DrawAllDoodads():
    global collisionsDetected
    DebugPrint("FNR State", publishedFNR, 0)
    DebugPrint("Throttle", publishedThrottle, 7)
    DebugPrint("Steering", publishedSteering, 2)
    DebugPrint("Collisions", collisionsDetected, 1)
    DebugPrint("CruiseControl", publishedCruiseControl, 5)
    DebugPrint("GetSteerValue", actualSteering, 4)
    collisionsDetected = 0

    if publishedCruiseControl == 1:
        throttleColor = (200,200,200)
    else:
        throttleColor = (0,204,0)
    rect = pygame.draw.rect(screen, throttleColor, (WIDTH - 205, 150, 200, 15), 2)
    rect = pygame.draw.rect(screen, throttleColor, (WIDTH - 205, 150, publishedThrottle*100*2.0/40, 15), 0)


def RefreshGUI():
    #screen.fill(BACKGROUND_COLOR)
    screen.blit(BACKGROUND_IMAGE, (0,0))
    screen.blit(LOGO_IMAGE, (WIDTH-237,0))
    DrawStaticObjects()
    DrawAllDoodads()

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
    targetAngle = float(actualSteering-HALF_MAX_STEERING_VALUE)
    if virtualMode == 1:
        targetAngle = float(publishedSteering -HALF_MAX_STEERING_VALUE)

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
            ) and lidarDataArray[i] < rangeConsidered:
            pointColor = (204,0,51)
            global collisionsDetected
            collisionsDetected+=1
        elif(targetAngle > -5 and targetAngle < 5):
            pointColor = (51,204,0)
            if (i > 90 and i < 450):
                if (abs(math.cos(math.radians(abs(theta)))*lidarDataArray[i])<CART_WIDTH):
                    pointColor = (204,0,51)
                    global collisionsDetected
                    collisionsDetected+=1


        else:
            pointColor = (51,204,0)
        """
        """
        #(setCircleOriginX,setCircleOriginY,setCircleInnerRadius,setCircleOuterRadius)

        #Draws Lidar Points
        cos_function = lidarDataArray[i]*(math.cos(math.radians(theta)))
        sin_function = lidarDataArray[i]*(math.sin(math.radians(theta)))
        DrawCircle(pointColor, SCALE_FACTOR*(X_AXIS_ORIGIN+cos_function), SCALE_FACTOR*(Y_AXIS_ORIGIN-sin_function), 3,3)

        #if crashData[1] != 0:
        if targetAngle > 5 or targetAngle < -5:
        #Draws Collision Circles
        #Draws Crash Distance Outer Circle
            cos_function = crashData[1]*(math.cos(math.radians(theta)))
            sin_function = crashData[1]*(math.sin(math.radians(theta)))
            DrawCircle((0,0,50), SCALE_FACTOR*(X_AXIS_ORIGIN+cos_function), SCALE_FACTOR*(Y_AXIS_ORIGIN-sin_function), 2,2)

        #Draws Crash Distance Inner Circle
        if (crashData[0] != -1):
            #if crashData[0] != 0:
            if targetAngle > 5 or targetAngle < -5:
                cos_function = crashData[0]*(math.cos(math.radians(theta)))
                sin_function = crashData[0]*(math.sin(math.radians(theta)))
                DrawCircle((0,0,50), SCALE_FACTOR*(X_AXIS_ORIGIN+cos_function), SCALE_FACTOR*(Y_AXIS_ORIGIN-sin_function), 2,2)

            #if crashData[2] != 0:
            if targetAngle > 5 or targetAngle < -5:
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
        DrawRectangle(CIRCLE_COLOR, (SCALE_FACTOR*(X_AXIS_ORIGIN-CART_WIDTH), 0), SCALE_FACTOR*CART_WIDTH*2, HEIGHT, 3)
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
        #DrawRectangle(TRUE_CIRCLE_COLOR, (SCALE_FACTOR*(X_AXIS_ORIGIN-CART_WIDTH), 0), SCALE_FACTOR*CART_WIDTH*2, HEIGHT, 3)
        pass
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
def ScreenDimensions():
    pressed = pygame.key.get_pressed()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            #pygame.quit()
            #sys.exit()
            pressed = pygame.key.get_pressed()
    if pressed[pygame.K_q]:
        pygame.quit()
        sys.exit()
    if pressed[pygame.K_f]:
        if not fullscreen:
            global fullscreen
            fullscreen = not fullscreen
            time.sleep(0.5)
            screen = pygame.display.set_mode((0,0), pyg.FULLSCREEN)
            global SCREEN_DIMENSIONS
            global WIDTH
            global HEIGHT
            SCREEN_DIMENSIONS = WIDTH, HEIGHT = screen.get_size()
            time.sleep(1)
        elif fullscreen:
            global fullscreen
            fullscreen = not fullscreen
            time.sleep(0.5)
            screen = pygame.display.set_mode((0,0), pyg.RESIZABLE)
            #global SCREEN_DIMENSIONS
            global WIDTH
            global HEIGHT
            SCREEN_DIMENSIONS = WIDTH, HEIGHT = screen.get_size()
            time.sleep(1)
    if pressed[pygame.K_LEFT]:
        time.sleep(1)
        global X_AXIS_ORIGIN
        X_AXIS_ORIGIN-=1

    elif pressed[pygame.K_RIGHT]:
        time.sleep(1)
        global X_AXIS_ORIGIN
        X_AXIS_ORIGIN+=1

    elif pressed[pygame.K_UP]:
        time.sleep(1)
        global Y_AXIS_ORIGIN
        Y_AXIS_ORIGIN-=1

    elif pressed[pygame.K_DOWN]:
        time.sleep(1)
        global Y_AXIS_ORIGIN
        Y_AXIS_ORIGIN+=1

    elif pressed[pygame.K_p]:
        time.sleep(1)
        global SCALE_FACTOR
        SCALE_FACTOR +=1

    elif pressed[pygame.K_o]:
        time.sleep(1)
        global SCALE_FACTOR
        SCALE_FACTOR -=1

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
    rospy.Subscriber("Get_Steering", UInt16, CallbackGetSteerValue)
    InitializeGUI()
    while not rospy.is_shutdown():
        ScreenDimensions()
        RefreshGUI()
        DrawTargetPath()
        DrawActualPath()
        DrawLidarData()
        pygame.display.update()
        time.sleep(0.01)

if __name__ == "__main__":
    print("TurnGUI Running")
    TurnGUI()
