#!/usr/bin/env python

import rospy, pygame, sys
from std_msgs.msg import UInt8, UInt16
MAX_STEERING_PUBLISH = 65535
def JoystickCtrls():
    #initialize the publishers
    pubFNR = rospy.Publisher("Set_FNR", UInt8, queue_size=1000)#100000
    pubSteering = rospy.Publisher("Set_Steering", UInt16, queue_size=1000)
    pubThrottle = rospy.Publisher("Set_Throttle", UInt16, queue_size=1000)
    rospy.init_node('JoystickCtrls', anonymous=True)
    pygame.init()
    controller = pygame.joystick.Joystick(0)
    controller.init()

    #Default Initialization values
    FNR = 1
    #if no new value = don't send new msg
    old_publishedThrottle = 0
    old_publishedSteering = 0
    old_FNR = 0

    fastMode = True
    cruiseControl = False
    steerControl = False

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

                if not controller.get_button(4) == 0: #LeftButton
                    cruiseControl = not cruiseControl
                if not controller.get_button(5) == 0: #RightButton
                    steerControl = not steerControl
                if not controller.get_button(7) == 0: #Right Trigger
                    fastMode = False
                if not controller.get_button(8) == 0: #Left Trigger
                    fastMode = False

        # Steering ctrls Triggers slowMode (right trigger to turn right, left trigger to turn left)
        leftSteering = controller.get_axis(2)
        if leftSteering > 0:
            fastMode = False
            if steering > -32765:
                steering -= 15
        rightSteering = controller.get_axis(5)
        if rightSteering > 0:
            fastMode = False
            if steering < 32765:
                steering += 15

        #Steering ctrls Joystick (right joystick left to turn left, right to turn right)
        if controller.get_axis(3) < -0.001 and steerControl == False:
            fastMode = True
            if fastMode == True:
                tiltValue = controller.get_axis(3)
                steering = 0.5*MAX_STEERING_PUBLISH*tiltValue
        elif controller.get_axis(3) > 0.001 and steerControl == False:
            fastMode = True
            if fastMode == True:
                tiltValue = controller.get_axis(3)
                steering = 0.5*MAX_STEERING_PUBLISH* tiltValue
        elif fastMode == True and steerControl == False:
            steering = 0

        #Throttle ctrls Joystick (left joystick up incr throttle, down decr throttle)
        if controller.get_axis(1) < -0.001 and cruiseControl == False:
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
        publishedSteering = int(0.5*MAX_STEERING_PUBLISH+ steering)
        if abs(publishedSteering - old_publishedSteering) > 300:
            pubSteering.publish(publishedSteering)
            old_publishedSteering = publishedSteering

if __name__  == "__main__":
    print "Joystick is now publishing data."
    print "FNR => UP, DOWN, RSHIFT"
    print "Steering => LEFT, RIGHT"
    print "Throttle => W, S"
    JoystickCtrls()
#i want to be able to store a value for throttle, FNR, steering
