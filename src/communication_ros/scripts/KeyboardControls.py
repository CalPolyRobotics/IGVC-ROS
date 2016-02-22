#!/usr/bin env python

import rospy
import pygame
import sys
from std_msgs.msg import UInt8, UInt16

def KeyboardCtrls():
	#initialize the publishers
	pubFNR = rospy.Publisher("Set_FNR", UInt8, queue_size=1000)#100000
	pubSteering = rospy.Publisher("Set_Steering", UInt16, queue_size=1000)
	pubThrottle = rospy.Publisher("Set_Throttle", UInt16, queue_size=1000)
	rospy.init_node('KeyboardCtrls', anonymous=True)
	
	#initialize pygame
	pygame.init()
	size = width,height = 420,320
	screen = pygame.display.set_mode(size)

	#if bored, more gui/ game stuff
	
	#set the message values
	throttle = 0.0
	FNR = 1
	steering = float(sys.maxint)/2.0

	#max for throttle & steering
	max_throttle = 100.0
	max_steering = 90.0	

	#rate multipliers for throttle & steering
	throttle_const = 200.0
	steering_const = 200.0
	#rate rates for throttle & steering
	throttle_delta = sys.maxint/(max_throttle * throttle_const)
	steering_delta = sys.maxint/(steering_const * max_steering)
	rate = rospy.Rate(30) #3600

    #if no new value = don't send new msg
    old_throttle = 0
    old_steering = 0
    old_FNR = 0

	while not rospy.is_shutdown():

		pressed = pygame.key.get_pressed()
		#hit the x, exit
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				sys.exit()	
			if event.type == pygame.KEYDOWN:	
				pressed = pygame.key.get_pressed()

		#Throttle ctrls(w incr throttle, s decr throttle)
		if pressed[pygame.K_w]:
            old_throttle = throttle
			if throttle < float(sys.maxint):
				throttle += throttle_delta
			else:
				throttle = sys.maxint
		elif pressed[pygame.K_s]:
			if throttle > 0:
				throttle -= throttle_delta
			else:
				throttle = 0
		else:
			throttle = 0

		#Steering ctrls(left to turn more to left, right to turn right)
		if pressed[pygame.K_LEFT]:
			old_steering = steering
			if steering > 0:
				steering -= steering_delta
			else:
				steering = 0
		elif pressed[pygame.K_RIGHT]:
			if steering < sys.maxint:
				steering += steering_delta
			else:
				steering = sys.maxint
		else:
			steering = float(sys.maxint)/2.0

		# 0 = forward, 1 = neutral, 2 = reverse
		if pressed[pygame.K_UP]:
			old_FNR = FNR
			FNR = 0
		elif pressed[pygame.K_DOWN]:
			old_FNR = FNR
			FNR = 2
		elif pressed[pygame.K_RSHIFT]:
			old_FNR = FNR
			FNR = 1
	
		#convert throttle to b/w 0 & 100, steering from 0 to 90	
		# 0 = no throttle, 100 = full throttle; 0 = all the way left, 90 = all the way right
        if FNR not old_FNR:
			print "FNR: " +  str(FNR)
			pubFNR.publish(FNR)
        
        if steering not old_steering:
			print "Steering:" + str(convert(steering, max_steering))
			pubSteering.publish(convert(steering, max_steering))

        if throttle not old_throttle:
			print "Throttle: " + str(convert(throttle,max_throttle))
			pubThrottle.publish(convert(throttle, max_throttle))

def convert(raw, max_range):
	return int( (float(raw)/float(sys.maxint)) * max_range)

if __name__  == "__main__":
	print "FNR => UP, DOWN, RSHIFT"
	print "Steering => LEFT, RIGHT"
	print "Throttle => W, S"
	KeyboardCtrls()
#i want to be able to store a value for throttle, FNR, steering
