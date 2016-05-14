#!/usr/bin/env python

import rospy, pygame, math, sys, serial, time
from std_msgs.msg import UInt8, UInt16, Float32MultiArray, UInt8MultiArray
from willitcrash import getCrashDistancesCartesian
from numpy import interp

steering = 0
SERIAL_ENABLE = 0#
USB_ENABLE = 0
OBSTACLE_THRESHOLD = 1
SCALE_FACTOR = 30
AXIS_CENTER = 20
CART_HEIGHT = 65*2.54/100
CART_WIDTH = 45/2*2.54/100 # Half of the cart width #45
MAX_STEERING_DEGREES = 25
MAX_STEERING_PUBLISH= 65535
TEXT_COLOR = (0, 51, 102)
color = (150,135,150)
lidarDataArray = []

publishedThrottle = 5
publishedSteering = 2000
FNR = 1

for i in range (0,540):
    lidarDataArray.append(1)

def drawText(screen, font, text, index):
    screen.blit(font.render(str(text), True, TEXT_COLOR), (200, 800 -  30*index))

def debugPrint(screen, font, text, title, index):
    screen.blit(font.render(str(text), True, TEXT_COLOR), (900, 800 -  30*index))
    screen.blit(font.render(str(title), True, TEXT_COLOR), (900, 800 -  60*index))

def callBackSteering(data):
    global steering
    try:
        byte1 = int(data.data[0]) << 8
    except:
        byte1 = 0
    try:
        byte2 = int(data.data[1])
    except:
        byte2 = 0
    steering = byte1 + byte2

def drawPath(screen, font, steering):
    circleCenterX = 0
    circleCenterY = 0
    circleInnerR = 0
    circleOuterR = 0
    #Left Turn Wheel Path
    if steering < 0:
        steerInvert = -1
    else:
        steerInvert = 1

    #SteeringAngle ranges from 0 to 0.16109 [rads] (0 to MAX_STEERING_DEGREES degrees)
    steeringAngle = abs(steering*math.pi*MAX_STEERING_DEGREES/180/32767.5)
    #If steering is 0, circle approximation with a very small steering angle
    if steeringAngle == 0:
        steeringAngle = 0.001
    circleInnerR = abs(CART_HEIGHT/(math.tan(steeringAngle)) - 
            abs(0.5*CART_HEIGHT/math.tan(steeringAngle)))
    circleOuterR = abs(CART_HEIGHT/math.sin(steeringAngle) 
            - abs(0.5*CART_HEIGHT/math.sin(steeringAngle))) + 2*CART_WIDTH

    math.sqrt(pow(circleOuterR+CART_WIDTH,2) + pow(CART_HEIGHT,2))
    circleCenterX = steerInvert*(circleInnerR+CART_WIDTH)
    circleCenterY = CART_HEIGHT
    #Draw The Inner and Outer Circles
    drawCircle(screen, (200,200,200), 
            AXIS_CENTER+steerInvert*(CART_WIDTH+circleInnerR), 
            AXIS_CENTER+CART_HEIGHT, circleInnerR)
    drawCircle(screen, (200,200,200), 
            AXIS_CENTER+steerInvert*(CART_WIDTH+circleInnerR), 
            AXIS_CENTER+CART_HEIGHT, circleOuterR)
    # Draws the center point of the Inner and Outer Circles
    circle = pygame.draw.circle(screen, (255,0,0), 
            (int(SCALE_FACTOR*(AXIS_CENTER+circleCenterX)), 
            int(SCALE_FACTOR*(AXIS_CENTER+circleCenterY))), 2, 2)

    drawText(screen, font, str(round(math.degrees(steeringAngle), 2)) + ' deg.', 3)
    return (circleCenterX, circleCenterY, circleInnerR, circleOuterR)

def drawCircle(screen, circleColor, xCoordinate, yCoordinate, radius):
    circle = pygame.draw.circle(screen, circleColor, 
            (int(SCALE_FACTOR*xCoordinate), 
            int(SCALE_FACTOR*yCoordinate)), 
            int(SCALE_FACTOR*radius), 3)

def drawBackground(screen, font):
    #Begin Redrawing the GUI
    screen.fill(color)

    #Cartesian Coordinate Axis
    #Draws Vertical Axis
    line = pygame.draw.line(screen, (0,0,0), 
            (SCALE_FACTOR*AXIS_CENTER, 
            SCALE_FACTOR*2*AXIS_CENTER), 
            (SCALE_FACTOR*AXIS_CENTER, 0))
    #Draws Horizontal Axis
    line = pygame.draw.line(screen, (0,0,0), 
            (0, SCALE_FACTOR*AXIS_CENTER), 
            (SCALE_FACTOR*2*AXIS_CENTER, 
            SCALE_FACTOR*AXIS_CENTER))
    #Boundary Box
    rect = pygame.draw.rect(screen, (0,0,0), 
            (0, 0, 2*SCALE_FACTOR*AXIS_CENTER, 
            2*SCALE_FACTOR*AXIS_CENTER), 2)

    # 5 Meter Line Markers
    for i in range (1,5):
        line = pygame.draw.line(screen, (10,10,10), 
                (SCALE_FACTOR*0.5*AXIS_CENTER, SCALE_FACTOR*(AXIS_CENTER-i*5)), 
                (SCALE_FACTOR*1.5*AXIS_CENTER, SCALE_FACTOR*(AXIS_CENTER-i*5)))
        screen.blit(font.render(str(i*5) + "m", True, TEXT_COLOR), 
                (SCALE_FACTOR*0.5*AXIS_CENTER, SCALE_FACTOR*(AXIS_CENTER-i*5)))

    #Golf Cart Rectangle
    rect = pygame.draw.rect(screen, (0,0,0), 
            (int(SCALE_FACTOR*(AXIS_CENTER-CART_WIDTH)), 
            int(SCALE_FACTOR*(AXIS_CENTER)), 
            int(SCALE_FACTOR*(2*CART_WIDTH)), 
            int(SCALE_FACTOR*CART_HEIGHT)), 0)

    #Joystick Summary GUI Text
    screen.blit(font.render('FNR Status:', True, TEXT_COLOR), (10, 800))
    screen.blit(font.render('Throttle Status:', True, TEXT_COLOR), (10, 800-30))
    screen.blit(font.render('Steering Status:', True, TEXT_COLOR), (10, 800-60))
    screen.blit(font.render('Wheel Angle:', True, TEXT_COLOR), (10, 800-90))

def callback(data):
    #global var
    #var = True
    global lidarDataArray
    for i in range(0, 540):
        #Multiply by 10 to convert meters to dm and multiplier for GUI scaling
        lidarDataArray[i] = data.data[i]
def callbackFNR(data):
    global FNR
    FNR = int(data)
def callbackThrottle(data):
    global publishedThrottle
    publishedThrottle = int(data)
#def callbackSteering(data):
    #global steering
    #steering = data

def TurnGUI():
    global steering
    #initialize the publishers
    pubLights = rospy.Publisher("Set_Lights", UInt16, queue_size=1000)
    rospy.init_node('TurnGUI', anonymous=True)
    rospy.Subscriber("Get_Steering", UInt8MultiArray, callBackSteering)
    rospy.Subscriber("lidar_scan_ranges", Float32MultiArray, callback)
    rospy.Subscriber("Set_FNR", UInt8, callbackFNR)
    rospy.Subscriber("Set_Throttle", UInt16, callbackThrottle)
    #rospy.Subscriber("Set_Steering", UInt16, callbackSteering)

    pygame.init()
    size = width, height = 1350, 825
    screen = pygame.display.set_mode(size)
    pygame.display.set_caption("IGVC HUD")
    screen.fill(color)

    #Default Initialization values
    FNR_print = "zero"
    throttle_print = "zero"
    steering_print = "zero"
    circleCenterX = 0
    circleCenterY = 0
    circleInnerR = 0
    circleInnerR = 0
    colorVariant = 0
    pygame.display.update()
    controller = pygame.joystick.Joystick(0)
    controller.init()

    font = pygame.font.SysFont('Arial', 20)
    #set the message values
    throttle = 0.0
    #FNR = 1

    #max for throttle & steering

    #if no new value = don't send new msg
    old_publishedThrottle = 0
    old_publishedSteering = 0
    old_FNR = 0
    old_num_data= 0
    steerHistory = []


    fastMode = True
    cruiseControl = False
    steerControl = False

    #Debug Mode Via Serial or USB
    if SERIAL_ENABLE == 1:
        ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            timeout=0.2)
    if USB_ENABLE == 1:
        pass

    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        #GUI Background & Live Data
        drawBackground(screen, font)

        # Draw Cart Path & Store Coordinates & Radii in pathParams Tuple
        pathParams = drawPath(screen, font, steering)

        #FNR Status GUI
        FNR_print = None
        FNRColor = (0,125,0)
        if FNR == 0:
            FNR_print = "Neutral"
            FNRColor = (150,150,0)
        if FNR == 1:
            FNR_print = "Forward"
            FNRColor = (0,125,0)
        if FNR == 2:
            FNR_print = "Backward"
            FNRColor = (150,0,0)
        rect = pygame.draw.rect(screen, FNRColor, (200, 800, 90, 25), 0)
        drawText(screen, font, FNR_print, 0)
        drawText(screen, font, str(publishedThrottle*100/40) + '%', 1)
        drawText(screen, font, steering, 2)

        #Throttle GUI
        throttleColor = (0,204,0)
        if cruiseControl == True:
            throttleColor = (200,200,0)
        else:
            throttleColor = (0,204,0)
        rect = pygame.draw.rect(screen, throttleColor, (265, 800-25, 250, 15), 2)
        rect = pygame.draw.rect(screen, throttleColor, (265, 800-25, publishedThrottle*100*2.5/40, 15), 0)
        #CruiseControl GUI
        if cruiseControl == True:
            rectColor = (100,0,0)
        else:
            rectColor = color
        rect = pygame.draw.rect(screen, rectColor, (725, 750, 225, 100), 0)
        #SteerControl GUI
        if steerControl == True:
            rectColor = (100,0,0)
        else:
            rectColor = color
        rect = pygame.draw.rect(screen,rectColor, (950, 750, 225, 100), 0)
        screen.blit(font.render("Steer Control: " + str(steerControl), True, TEXT_COLOR), (950, 800))
        screen.blit(font.render("Cruise Control: " + str(cruiseControl), True, TEXT_COLOR), (750, 800))


        #Lidar Data GUI
        if True:
            crashDistances = getCrashDistancesCartesian(pathParams[0], pathParams[1], pathParams[2], pathParams[3])
            pointColor = (0,0,0)
            obstaclesDetected = 0
            for i in range(0, 540):
                crashData = crashDistances[i]
                theta = (i-90)*90/180
                pointColor = (255-colorVariant,0,0)
                if lidarDataArray[i] < 0.05:
                    colorVariant = int(lidarDataArray[i]*255/20)
                    pointColor = (0,0,255)
                elif (
                    ((crashData[0] == -1) and (lidarDataArray[i] < crashData[1])) or
                    ((crashData[0] != -1) and ((lidarDataArray[i] < crashData[0]) or 
                    ((lidarDataArray[i] > crashData[2]) and (lidarDataArray[i] < crashData[1]))))
                        ):
                    colorVariant = int(lidarDataArray[i]*255/20)
                    obstaclesDetected += lidarDataArray[i]
                else:
                    pointColor = (0,255-colorVariant,0)
                #Draws Lidar Points
                cos_function = lidarDataArray[i]*(math.cos(math.radians(theta)))
                sin_function = lidarDataArray[i]*(math.sin(math.radians(theta)))
                drawCircle(screen, pointColor, AXIS_CENTER+cos_function, AXIS_CENTER-sin_function, 3.0/SCALE_FACTOR)

                #Draws Crash Distance Outer Circle
                cos_function = crashData[1]*(math.cos(math.radians(theta)))
                sin_function = crashData[1]*(math.sin(math.radians(theta)))
                drawCircle(screen, (255,255,255), AXIS_CENTER+cos_function, AXIS_CENTER-sin_function, 3.0/SCALE_FACTOR)

                #Draws Crash Distance Inner Circle
                if (crashData[0] != -1):
                    cos_function = crashData[0]*(math.cos(math.radians(theta)))
                    sin_function = crashData[0]*(math.sin(math.radians(theta)))
                    drawCircle(screen, (255,255,255), AXIS_CENTER+cos_function, AXIS_CENTER-sin_function, 3.0/SCALE_FACTOR)

                    cos_function = crashData[2]*(math.cos(math.radians(theta)))
                    sin_function = crashData[2]*(math.sin(math.radians(theta)))
                    drawCircle(screen, (255,255,255), AXIS_CENTER+cos_function, AXIS_CENTER-sin_function, 3.0/SCALE_FACTOR)
            if obstaclesDetected > OBSTACLE_THRESHOLD:
                if SERIAL_ENABLE == 1:
                    ser.write("[setLED,8,1]")
                    time.sleep(0.05)
                    ser.readline()
                    debugPrint(screen, font, obstaclesDetected, "Obstacles", 1)
                elif USB_ENABLE == 1:
                    pubLights.publish(0x0080)
                else:
                    debugPrint(screen, font, obstaclesDetected, "Obstacles", 1)
            else:
                if SERIAL_ENABLE == 1:
                    ser.write("[setLED,8,0]")
                    time.sleep(0.05)
                    ser.readline()
                    debugPrint(screen, font, obstaclesDetected, "Obstacles", 1)
                elif USB_ENABLE == 1:
                    pubLights.publish(0)
                else:
                    debugPrint(screen, font, obstaclesDetected, "Obstacles", 1)

        #End Redrawing The GUI and Update
        pygame.display.update()

if __name__  == "__main__":
    print "FNR => UP, DOWN, RSHIFT"
    print "Steering => LEFT, RIGHT"
    print "Throttle => W, S"
    TurnGUI()
#i want to be able to store a value for throttle, FNR, steering
