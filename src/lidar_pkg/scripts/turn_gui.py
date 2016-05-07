#!/usr/bin/env python

import rospy, pygame, math, sys
from std_msgs.msg import UInt8, UInt16, Float32MultiArray

multiplier = 3
num = 1.0
var = False
array = []
for i in range (0,540):
    array.append(1)

def callback(data):
    global var
    var = True
    global num
    num = 100*data.data[270];
    global array
    for i in range(0, 540):
        #Multiply by 10 to convert meters to dm and multiplier for GUI scaling
        array[i] = 10*data.data[i]*multiplier

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
    font = pygame.font.SysFont('Arial', 20)
    color = (115, 115, 115)
    black = (0, 0, 0)
    screen.fill(color)

    # GUI SETUP
    scaling_factor = 2.5
    cart_height = int(65*2.54*multiplier/10)
    cart_width = int(45/2*2.54*multiplier/10) # Half of the cart width
    axis_center = 625

    #Default Initialization values
    FNR_print = "zero"
    throttle_print = "zero"
    steering_print = "zero"
    radius = 0
    outerRadius = 0
    degrees = 0
    pygame.display.update()
    controller = pygame.joystick.Joystick(0)
    controller.init()

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

        # Steering ctrls slowMode (right trigger to turn right, left trigger to turn left)
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
        if controller.get_axis(3) < -1*0.10:
            fastMode = True
            if fastMode == True:
                tiltValue = round(controller.get_axis(3), 1)
                steering = 0.5*max_steering* tiltValue
        elif controller.get_axis(3) > 0.10:
            fastMode = True
            if fastMode == True:
                tiltValue = round(controller.get_axis(3), 1)
                steering = 0.5*max_steering* tiltValue
        elif fastMode == True:
            steering = 0

        #Throttle ctrls(left joystick up incr throttle, down decr throttle)
        if controller.get_axis(1) < -1*0.05 and cruiseControl == False:
            tiltValue = round(-1*controller.get_axis(1), 1)
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
        if abs(publishedSteering - old_publishedSteering) > 1000:
            pubSteering.publish(publishedSteering)
            old_publishedSteering = publishedSteering

        #Begin Redrawing the GUI
        screen.fill(color)

        #Cartesian Coordinate Axis
        line = pygame.draw.line(screen, (0,0,0), (axis_center, axis_center+axis_center), 
                (axis_center, axis_center-axis_center))
        line = pygame.draw.line(screen, (0,0,0), (axis_center-axis_center, axis_center), 
                (axis_center+axis_center, axis_center))

        #Left Turn Wheel Path
        if steering < 0:
            degrees = (steering/32767.5)*-35 #32767 Steering Value, 35 degree approximation
            if math.tan(degrees) >  0:
                if (math.tan(math.radians(degrees))) > 0:
                    radius = int(abs(cart_height/(math.tan(math.radians(degrees))) 
                            - abs(0.5*cart_height/math.tan(math.radians(degrees))))*2.54)
                    outerRadius = int(abs(cart_height/math.sin(math.radians(degrees)) 
                            - abs(0.5*cart_height/math.sin(math.radians(degrees))))*2.54)
            #Draw The Left Turn Wheel Path
            circle = pygame.draw.circle(screen, (255,255,255), 
                    (axis_center-cart_width-radius,axis_center+cart_height), radius, 3)
            circle = pygame.draw.circle(screen, (255,255,255), 
                    (axis_center-cart_width-radius,axis_center+cart_height), outerRadius+2*cart_width, 3)

        #Center Wheel Path
        if steering < 35 and steering > -35:
            rect = pygame.draw.rect(screen, (255,255,255), 
                    (axis_center-cart_width, 0, 2*cart_width,1000), 3)

        #Right Turn Wheel Path
        if steering > 0:
            degrees = (steering/32767.5)*35 #32767 Steering Value, 35 degree approximation
            if math.tan(degrees) > 0:
                if (math.tan(math.radians(degrees))) > 0:
                    radius = int((cart_height/(math.tan(math.radians(degrees))) - 
                            0.5*cart_height/math.tan(math.radians(degrees)))*2.54)
                    outerRadius = int((cart_height/math.sin(math.radians(degrees)) - 
                            0.5*cart_height/math.sin(math.radians(degrees)))*2.54)
            #Draw The Right Turn Wheel Path
            circle = pygame.draw.circle(screen, (255,255,255), 
                    (axis_center+cart_width+radius,axis_center+cart_height), radius, 3)
            circle = pygame.draw.circle(screen, (255,255,255), 
                    (axis_center+cart_width+radius,axis_center+cart_height), outerRadius+2*cart_width, 3)

        #Golf Cart Rectangle
        rect = pygame.draw.rect(screen, (0,0,0), 
                (axis_center-cart_width, axis_center, 2*cart_width, cart_height), 4)

        # 5 Meter Line Markers
        line = pygame.draw.line(screen, (10,10,10), 
                (axis_center-0.5*axis_center, axis_center-(multiplier*500/10)), 
                (0.5*axis_center+axis_center, axis_center-(multiplier*500/10)))
        line = pygame.draw.line(screen, (10,10,10), 
                (axis_center-0.5*axis_center, axis_center-(multiplier*2*500/10)), 
                (0.5*axis_center+axis_center, axis_center-(multiplier*2*500/10)))
        line = pygame.draw.line(screen, (10,10,10), 
                (axis_center-0.5*axis_center, axis_center-(multiplier*3*500/10)), 
                (0.5*axis_center+axis_center, axis_center-(multiplier*3*500/10)))
        line = pygame.draw.line(screen, (10,10,10), 
                (axis_center-0.5*axis_center, axis_center-(multiplier*4*500/10)), 
                (0.5*axis_center+axis_center, axis_center-(multiplier*4*500/10)))


        #Joystick Summary GUI Text
        myColor = (0, 51, 102)
        screen.blit(font.render('Joystick Summary:', True, myColor), (10, 10))
        screen.blit(font.render('FNR Status:', True, myColor), (10, 40))
        screen.blit(font.render('Throttle Status:', True, myColor), (10, 70))
        screen.blit(font.render('Steering Status:', True, myColor), (10, 100))

        #FNR Status GUI
        FNR_print = None
        if FNR == 0:
            FNR_print = "Neutral"
        if FNR == 1:
            FNR_print = "Forward"
        if FNR == 2:
            FNR_print = "Backward"
        throttle_print = str(publishedThrottle*100/40) + '%'
        steering_print = str(steering) + ''
        screen.blit(font.render(FNR_print, True, myColor), (175, 40))
        screen.blit(font.render(throttle_print, True, myColor), (175, 70))
        screen.blit(font.render(steering_print, True, myColor), (175, 100))

        #Throttle GUI
        rect = pygame.draw.rect(screen, (0,204,0), (250, 75, 250, 15), 2)
        rect = pygame.draw.rect(screen, (0,204,0), (250, 75, publishedThrottle*100*2.5/40, 15), 0)

        #Lidar Data GUI
        if var == True:
            for i in range(0, 540):
                theta = (i-90)*90/180
                cos_function = int(array[i]*(math.cos(math.radians(theta))))
                sin_function = int(array[i]*(math.sin(math.radians(theta))))
                circle = pygame.draw.circle(screen, (255,0,0), (axis_center+cos_function, axis_center-sin_function), 2, 2)

        #End Redrawing The GUI and Update
        pygame.display.update()

if __name__  == "__main__":
    print "FNR => UP, DOWN, RSHIFT"
    print "Steering => LEFT, RIGHT"
    print "Throttle => W, S"
    JoystickCtrls()
#i want to be able to store a value for throttle, FNR, steering
