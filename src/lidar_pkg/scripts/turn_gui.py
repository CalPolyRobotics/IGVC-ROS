#!/usr/bin/env python

import rospy, pygame, math, sys
from std_msgs.msg import UInt8, UInt16, Float32MultiArray
from willitcrash import getCrashDistancesCartesian

SCALE_FACTOR = 50
AXIS_CENTER = 10
CART_HEIGHT = 65*2.54/100
CART_WIDTH = 45/2*2.54/100 # Half of the cart width
TEXT_COLOR = (0, 51, 102)
color = (115, 115, 115)
var = False
lidarDataArray = []
for i in range (0,540):
    lidarDataArray.append(1)

def drawText(screen, font, text, index):
    screen.blit(font.render(str(text), True, TEXT_COLOR), (800, 10 + 30*index))

def drawPath(screen, font, steering):
    circleCenterX = 0
    circleCenterY = 0
    circleInnerCenterR = 0
    circleOuterCenterR = 0
    #Left Turn Wheel Path
    if steering < 0:
        steerInvert = -1
    else:
        steerInvert = 1

    #SteeringAngle ranges from 0 to 0.16109 [rads] (0 to 35 degrees)
    steeringAngle = abs(steering*math.pi*35/180/32767.5)
    #Center Wheel Path
    if steeringAngle < math.radians(0.01):
        rect = pygame.draw.rect(screen, (255,255,255), (SCALE_FACTOR*(AXIS_CENTER-CART_WIDTH), 0, 2*SCALE_FACTOR*CART_WIDTH,1000), 3)
    else: #if steeringAngle != 0:
        circleInnerCenterR = abs(CART_HEIGHT/(math.tan(steeringAngle)) - 
                abs(0.5*CART_HEIGHT/math.tan(steeringAngle)))

        circleOuterCenterR = abs(CART_HEIGHT/math.sin(steeringAngle) 
                - abs(0.5*CART_HEIGHT/math.sin(steeringAngle))) + 2*CART_WIDTH
        math.sqrt(pow(circleOuterCenterR+CART_WIDTH,2) + pow(CART_HEIGHT,2))
        circleCenterX = steerInvert*(circleInnerCenterR+CART_WIDTH)
        circleCenterY = CART_HEIGHT
        #Draw The Inner and Outer Circles
        drawCircle(screen, (255,255,255), AXIS_CENTER+steerInvert*(CART_WIDTH+circleInnerCenterR), AXIS_CENTER+CART_HEIGHT, circleInnerCenterR)
        drawCircle(screen, (255,255,255), AXIS_CENTER+steerInvert*(CART_WIDTH+circleInnerCenterR), AXIS_CENTER+CART_HEIGHT, circleOuterCenterR)
        # Draws the center point of the Inner and Outer Circles
        circle = pygame.draw.circle(screen, (255,0,0), 
                (int(SCALE_FACTOR*(AXIS_CENTER+circleCenterX)), 
                int(SCALE_FACTOR*(AXIS_CENTER+circleCenterY))), 2, 2)

    #else: #if steeringAngle == 0

    drawText(screen, font, steeringAngle, 3)
    return (circleCenterX, circleCenterY, circleInnerCenterR, circleOuterCenterR)

def drawCircle(screen, circleColor, xCoordinate, yCoordinate, radius):
    circle = pygame.draw.circle(screen, circleColor, (int(SCALE_FACTOR*xCoordinate), int(SCALE_FACTOR*yCoordinate)), int(SCALE_FACTOR*radius), 3)

def drawBackground(screen, font):
    #Begin Redrawing the GUI
    screen.fill(color)

    #Cartesian Coordinate Axis
    #Draws Vertical Axis
    line = pygame.draw.line(screen, (0,0,0), (SCALE_FACTOR*AXIS_CENTER, SCALE_FACTOR*2*AXIS_CENTER), 
            (SCALE_FACTOR*AXIS_CENTER, 0))
    #Draws Horizontal Axis
    line = pygame.draw.line(screen, (0,0,0), (0, SCALE_FACTOR*AXIS_CENTER), 
            (SCALE_FACTOR*2*AXIS_CENTER, SCALE_FACTOR*AXIS_CENTER))
    #Boundary Box
    rect = pygame.draw.rect(screen, (0,0,0), (0, 0, 2*SCALE_FACTOR*AXIS_CENTER, 2*SCALE_FACTOR*AXIS_CENTER), 4)

    # 5 Meter Line Markers
    for i in range (1,5):
        line = pygame.draw.line(screen, (10,10,10), 
            (SCALE_FACTOR*0.5*AXIS_CENTER, SCALE_FACTOR*(AXIS_CENTER-i*5)), 
            (SCALE_FACTOR*1.5*AXIS_CENTER, SCALE_FACTOR*(AXIS_CENTER-i*5)))

    #Golf Cart Rectangle
    rect = pygame.draw.rect(screen, (0,0,0), 
            (int(SCALE_FACTOR*(AXIS_CENTER-CART_WIDTH)), int(SCALE_FACTOR*(AXIS_CENTER)), int(SCALE_FACTOR*(2*CART_WIDTH)), int(SCALE_FACTOR*CART_HEIGHT)), 4)

    #Joystick Summary GUI Text
    screen.blit(font.render('Joystick Summary:', True, TEXT_COLOR), (600, 10))
    screen.blit(font.render('FNR Status:', True, TEXT_COLOR), (600, 40))
    screen.blit(font.render('Throttle Status:', True, TEXT_COLOR), (600, 70))
    screen.blit(font.render('Steering Status:', True, TEXT_COLOR), (600, 100))


def callback(data):
    global var
    var = True
    global lidarDataArray
    for i in range(0, 540):
        #Multiply by 10 to convert meters to dm and multiplier for GUI scaling
        #lidarDataArray[i] = 100*data.data[i]*multiplier
        lidarDataArray[i] = data.data[i]

def JoystickCtrls():
    #initialize the publishers
    pubFNR = rospy.Publisher("Set_FNR", UInt8, queue_size=1000)#100000
    pubSteering = rospy.Publisher("Set_Steering", UInt16, queue_size=1000)
    pubThrottle = rospy.Publisher("Set_Throttle", UInt16, queue_size=1000)
    rospy.init_node('JoystickCtrls', anonymous=True)
    rospy.Subscriber("lidar_scan_ranges", Float32MultiArray, callback)

    pygame.init()
    size = width, height = 1250, 750
    screen = pygame.display.set_mode(size)
    pygame.display.set_caption("HUD")
    screen.fill(color)


    #Default Initialization values
    FNR_print = "zero"
    throttle_print = "zero"
    steering_print = "zero"
    radius = 0
    outerRadius = 0
    circleCenterX = 0
    circleCenterY = 0
    circleInnerCenterR = 0
    circleInnerCenterR = 0
    pygame.display.update()
    controller = pygame.joystick.Joystick(0)
    controller.init()

    font = pygame.font.SysFont('Arial', 20)
    #set the message values
    throttle = 0.0
    FNR = 1
    steering = float(sys.maxint)/2.0

    #max for throttle & steering
    max_throttle = 40.0
    max_steering = 65535

    #if no new value = don't send new msg
    old_publishedThrottle = 0
    old_publishedSteering = 0
    old_FNR = 0

    fastMode = True
    cruiseControl = False
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.JOYBUTTONDOWN:
                #Foward and background ctrls(right joystick up and down is forward & backward)
                if not controller.get_button(2) == 0: #Button X
                    old_FNR = FNR
                    if cruiseControl:
                        cruiseControl = not cruiseControl
                    FNR = 0
                if not controller.get_button(0) == 0: #Button A
                    old_FNR = FNR
                    if cruiseControl:
                        cruiseControl = not cruiseControl
                    FNR = 2
                if not controller.get_button(1) == 0: #Button X
                    old_FNR = FNR
                    if cruiseControl:
                        cruiseControl = not cruiseControl
                    FNR = 0
                if not controller.get_button(3) == 0: #Button Y
                    old_FNR = FNR
                    if cruiseControl:
                        cruiseControl = not cruiseControl
                    FNR = 1

                if not controller.get_button(5) == 0: #Right Button
                    cruiseControl = not cruiseControl
                if not controller.get_button(7) == 0: #Right Trigger
                    fastMode = False
                if not controller.get_button(8) == 0: #Left Trigger
                    fastMode = False

        # Steering ctrls Triggers slowMode (right trigger to turn right, left trigger to turn left)
        leftSteering = controller.get_axis(2)
        if leftSteering > 0.05:
            fastMode = False
            if steering > -32765:
                steering -= 15
        rightSteering = controller.get_axis(5)
        if rightSteering > 0.05:
            fastMode = False
            if steering < 32765:
                steering += 15

        #Steering ctrls Continued (right joystick left to turn left, right to turn right)
        #if controller.get_axis(3) < -1*0.01:
        if controller.get_axis(3) < 0:
            fastMode = True
            if fastMode == True:
                tiltValue = controller.get_axis(3)
                steering = 0.5*max_steering*tiltValue
        #elif controller.get_axis(3) > 0.01:
        elif controller.get_axis(3) > 0:
            fastMode = True
            if fastMode == True:
                tiltValue = controller.get_axis(3)
                steering = 0.5*max_steering* tiltValue
        elif fastMode == True:
            steering = 0

        #Throttle ctrls(left joystick up incr throttle, down decr throttle)
        if controller.get_axis(1) < -1*0.05 and cruiseControl == False:
            #tiltValue = round(-1*controller.get_axis(1), 1)
            tiltValue = -1*controller.get_axis(1)
            throttle = 40.0 * tiltValue
        elif cruiseControl == False:
            throttle = 0

        #Publish The Throttle
        publishedThrottle = int(throttle)
        if not publishedThrottle == old_publishedThrottle:
            pubThrottle.publish(publishedThrottle)
            old_publishedThrottle = publishedThrottle

        #Publish The FNR
        if not FNR == old_FNR:
            old_FNR = FNR
            pubFNR.publish(FNR)

        #Publish The Steering
        publishedSteering = int(0.5*max_steering + steering)
        if abs(publishedSteering - old_publishedSteering) > 100:
            pubSteering.publish(publishedSteering)
            old_publishedSteering = publishedSteering

        #GUI Background & Live Data
        drawBackground(screen, font)
        # Draw Cart Path & Store Coordinates & Radii in pathParams Tuple
        pathParams = drawPath(screen, font, steering)

        #Lidar Data GUI
        #if var == True:
        if True:
            drawText(screen, font, pathParams[0], 5)
            drawText(screen, font, pathParams[1], 6)
            drawText(screen, font, pathParams[2], 7)
            drawText(screen, font, pathParams[3], 8)

            crashDistances = getCrashDistancesCartesian(pathParams[0], pathParams[1], pathParams[2], pathParams[3])
            pointColor = (0,0,0)
            for i in range(0, 540):
                crashData = crashDistances[i]
                theta = (i-90)*90/180
                if steering > 0:
                    if (
                        ((crashData[0] == -1) and (lidarDataArray[i] < crashData[1])) or
                            ((crashData[0] != -1) and (lidarDataArray[i] < crashData[0]))
                            ):
                        pointColor = (255,0,0)
                    else:
                        pointColor = (0,255,0)
                elif steering < 0:
                    if (
                        lidarDataArray[i] > crashData[2]
                        ):
                        pointColor = (0,255,255)
                    else:
                        pointColor = (255,0,255)
                #Draws Lidar Points
                cos_function = lidarDataArray[i]*(math.cos(math.radians(theta)))
                sin_function = lidarDataArray[i]*(math.sin(math.radians(theta)))
                drawCircle(screen, pointColor, AXIS_CENTER+cos_function, AXIS_CENTER-sin_function, 3.0/SCALE_FACTOR)

                #Draws Crash Distance Outer Circle
                cos_function = crashData[1]*(math.cos(math.radians(theta)))
                sin_function = crashData[1]*(math.sin(math.radians(theta)))
                drawCircle(screen, (0,0,255), AXIS_CENTER+cos_function, AXIS_CENTER-sin_function, 3.0/SCALE_FACTOR)

                #Draws Crash Distance Inner Circle
                if (crashData[0] != -1):
                    cos_function = crashData[0]*(math.cos(math.radians(theta)))
                    sin_function = crashData[0]*(math.sin(math.radians(theta)))
                    drawCircle(screen, (0,0,255), AXIS_CENTER+cos_function, AXIS_CENTER-sin_function, 3.0/SCALE_FACTOR)

                    cos_function = crashData[2]*(math.cos(math.radians(theta)))
                    sin_function = crashData[2]*(math.sin(math.radians(theta)))
                    drawCircle(screen, (0,0,255), AXIS_CENTER+cos_function, AXIS_CENTER-sin_function, 3.0/SCALE_FACTOR)


                

        #FNR Status GUI
        FNR_print = None
        if FNR == 0:
            FNR_print = "Neutral"
        if FNR == 1:
            FNR_print = "Forward"
        if FNR == 2:
            FNR_print = "Backward"
        drawText(screen, font, FNR_print, 0)
        drawText(screen, font, str(publishedThrottle*100/40) + '%', 1)
        drawText(screen, font, steering, 2)

        #Throttle GUI
        rect = pygame.draw.rect(screen, (0,204,0), (250, 75, 250, 15), 2)
        rect = pygame.draw.rect(screen, (0,204,0), (250, 75, publishedThrottle*100*2.5/40, 15), 0)

        #End Redrawing The GUI and Update
        pygame.display.update()

if __name__  == "__main__":
    print "FNR => UP, DOWN, RSHIFT"
    print "Steering => LEFT, RIGHT"
    print "Throttle => W, S"
    JoystickCtrls()
#i want to be able to store a value for throttle, FNR, steering
