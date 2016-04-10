#!/usr/bin/env python

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
	
	#initialize pygame & joystick
	pygame.init()
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
				steering -= 0.15
		"""
		elif leftSteering < -0.05 and abs(1 - leftSteering) > 0.05:
			fastMode = False
			if steering > -32765:
				steering -= 0.05
		"""
			
		rightSteering = controller.get_axis(5)
		if rightSteering > 0.05:
			fastMode = False
			if steering < 32765:
				steering += 0.15
		"""
		elif rightSteering < -0.05 and abs(1 - rightSteering) > 0.05:
			fastMode = False
			if steering < 32765:
				steering += 0.05
		"""
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

		publishedThrottle = int(throttle)
		if not publishedThrottle == old_publishedThrottle:
			pubThrottle.publish(publishedThrottle)
			old_publishedThrottle = publishedThrottle

		if not FNR == old_FNR:
			old_FNR = FNR
			pubFNR.publish(FNR)

		publishedSteering = int(0.5*max_steering + steering)
		if abs(publishedSteering - old_publishedSteering) > 1000:
			pubSteering.publish(publishedSteering)
			old_publishedSteering = publishedSteering

		
if __name__  == "__main__":
	print "FNR => UP, DOWN, RSHIFT"
	print "Steering => LEFT, RIGHT"
	print "Throttle => W, S"
	KeyboardCtrls()
#i want to be able to store a value for throttle, FNR, steering
