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
	size = width,height = 420,320
	screen = pygame.display.set_mode(size)
	color = (200, 200, 200)
	black = (0, 0, 0)
	pygame.display.set_caption("HUD")
	screen.fill(color)

	font = pygame.font.SysFont('Arial', 20)
	#screen.blit(font.render('Hello!', True, (255, 0, 0)))
	screen.blit(font.render('Joystick Summary', True, (255,0,0)), (10, 10))
	screen.blit(font.render('FNR Status: ', True, (255,0,0)), (10, 40))
	screen.blit(font.render('Throttle Status: ', True, (255,0,0)), (10, 70))
	screen.blit(font.render('Steering Status: ', True, (255,0,0)), (10, 100))
	
	FNR_print = "zero"
	throttle_print = "zero"
	steering_print = "zero"	


	pygame.display.update()

	controller = pygame.joystick.Joystick(0)
	controller.init()

	#if bored, more gui/ game stuff
	
	#set the message values
	throttle = 0.0
	FNR = 1
	steering = float(sys.maxint)/2.0

	#max for throttle & steering
	max_throttle = 40.0
	max_steering = 65535

	#rate multipliers for throttle & steering
	throttle_const = 200.0
	steering_const = 2.0
	#rate rates for throttle & steering
	throttle_delta = sys.maxint/(max_throttle * throttle_const)
	steering_delta = sys.maxint/(steering_const * max_steering)
	rate = rospy.Rate(30) #3600

	#if no new value = don't send new msg
	old_throttle = 0
	old_publishedThrottle = 0
	old_steering = 0
	old_publishedSteering = 0
	old_FNR = 0

	while not rospy.is_shutdown():
		buttonPressed = None
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				sys.exit()
			if event.type == pygame.JOYBUTTONDOWN:
				if not controller.get_button(2) == 0:
					buttonPressed = 'X'
				if not controller.get_button(0) == 0:
					buttonPressed = 'A'
				if not controller.get_button(1) == 0:
					buttonPressed = 'B'
				if not controller.get_button(3) == 0:
					buttonPressed = 'Y'
		#Throttle ctrls(left joystick up incr throttle, down decr throttle)
		if controller.get_axis(1) < -1*0.05:	
			old_throttle = throttle
			tiltValue = round(-1*controller.get_axis(1), 1)
			throttle = 40.0 * tiltValue
		else:
			throttle = 0

		#Steering ctrls(right joystick left to turn left, right to turn right)
		if controller.get_axis(3) < -1*0.10:
			old_steering = steering
			tiltValue = round(controller.get_axis(3), 1)
			steering = 0.5*max_steering* tiltValue
		elif controller.get_axis(3) > 0.10:
			old_steering = steering
			tiltValue = round(controller.get_axis(3), 1)
			steering = 0.5*max_steering* tiltValue
		else:
			steering = 0
				
		#Foward and background ctrls(right joystick up and down is forward & backward)
		# FNR Values: 0 = forward, 1 = neutral, 2 = reverse
		if buttonPressed == 'Y':
			old_FNR = FNR
			FNR = 0
			FNR_print = "0"
			#rect = pygame.draw.rect(screen, (black), (175, 75, 200, 100), 2)
			screen.fill(color)

			screen.blit(font.render(FNR_print, True, (255,0,0)), (200, 40))
			pygame.display.update()


		elif buttonPressed == 'A':
			old_FNR = FNR
			FNR = 2
			FNR_print = "2"
			#rect = pygame.draw.rect(screen, (black), (175, 75, 200, 100), 2)

			screen.fill(color)

			screen.blit(font.render(FNR_print, True, (255,0,0)), (200, 40))
			pygame.display.update()

		elif buttonPressed == 'X' or buttonPressed == 'Y':
			old_FNR = FNR
			FNR = 1
			FNR_print = "1"
			#rect = pygame.draw.rect(screen, (black), (175, 75, 200, 100), 2)
			screen.fill(color)

			screen.blit(font.render(FNR_print, True, (255,0,0)), (200, 40))
			pygame.display.update()


		#convert throttle to b/w 0 & 40, steering from 0 to 65535
		# 0 = no throttle, 100 = full throttle; 0 = all the way left, 90 = all the way right
		if not FNR == old_FNR:
			old_FNR = FNR
			pubFNR.publish(FNR)

		publishedSteering = int(0.5*max_steering + steering)
		if not abs(publishedSteering - old_publishedSteering) < 1000:
			pubSteering.publish(publishedSteering)
			old_publishedSteering = publishedSteering

		publishedThrottle = int(throttle)
		if not publishedThrottle == old_publishedThrottle:
			pubThrottle.publish(publishedThrottle)
			old_publishedThrottle = publishedThrottle
		
		#screen.blit(font.render(FNR_print, True, (255,0,0)), (200, 40))
		#screen.blit(font.render(throttle_print, True, (255,0,0)), (200, 70))
		#screen.blit(font.render(steering_print, True, (255,0,0)), (200, 100))
		#pygame.display.update()


if __name__  == "__main__":
	print "FNR => UP, DOWN, RSHIFT"
	print "Steering => LEFT, RIGHT"
	print "Throttle => W, S"
	KeyboardCtrls()
#i want to be able to store a value for throttle, FNR, steering
