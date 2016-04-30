#!/usr/bin/env python

import rospy
import pygame
import math
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
	size = width, height = 1000, 750
	screen = pygame.display.set_mode(size)
	color = (115, 115, 115)
	black = (0, 0, 0)
	pygame.display.set_caption("HUD")
	screen.fill(color)
	font = pygame.font.SysFont('Arial', 20)
	screen.blit(font.render('Joystick Summary:', True, (0,51,102)), (10, 10))
	screen.blit(font.render('FNR Status:', True, (0,51,102)), (10, 40))
	screen.blit(font.render('Throttle Status:', True, (0,51,102)), (10, 70))
	screen.blit(font.render('Steering Status:', True, (0,51,102)), (10, 100))
	rect = pygame.draw.rect(screen, (0,0,0), (250, 75, 250, 15), 2)	
	rect = pygame.draw.rect(screen, (0,204,0), (250, 75, 250, 15), 0)	
	radius = 1
	degrees = 0



	FNR_print = "zero"
	throttle_print = "zero"
	steering_print = "zero"
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
		screen.fill(color)
		
		#Cartesian Coordinate Axis
		line = pygame.draw.line(screen, (0,0,0), (500, 500+250), (500, 500-250))
		line = pygame.draw.line(screen, (0,0,0), (500-250, 500), (500+250, 500))


		#Golf Cart Paths
		if steering < 0:
			degrees = (steering/32767.5)*-35 #32767
			if math.tan(degrees) >  0:
				if (math.tan(math.radians(degrees))) > 0:
					radius = abs(int(100+(65/(math.tan(math.radians(degrees))))))
			circle = pygame.draw.circle(screen, (255,0,0), (500-50-radius,500+150), radius, 3)
			# Other Tire
			circle = pygame.draw.circle(screen, (255,0,0), (500-50-radius,500+150), radius+100, 3)


	
		if steering == 0:
			rect = pygame.draw.rect(screen, (255,0,0), (500-50, 000, 100,1000), 3)
			#circle = pygame.draw.circle(screen, (255,0,0), (500-50-10000,500+150), 10000, 3)
			#circle = pygame.draw.circle(screen, (255,0,0), (500+50+10000,500+150), 10000, 3)


		if steering > 0:
			degrees = (steering/32767.5)*35 #32767
			if math.tan(degrees) > 0:
				if (math.tan(math.radians(degrees))) > 0:
					radius = int(100+(65/(math.tan(math.radians(degrees)))) - 0.5*65/math.tan(math.radians(degrees)))
			circle = pygame.draw.circle(screen, (255,0,0), (500+50+radius,500+150), radius, 3)
			# Other Tires
			circle = pygame.draw.circle(screen, (255,0,0), (500+50+radius,500+150), radius+100, 3)
		
		#Golf Cart
		rect = pygame.draw.rect(screen, (255,255,255), (500-50, 500, 100, 150), 4)	


		myColor = (0, 51, 102)
		screen.blit(font.render('Joystick Summary:', True, myColor), (10, 10))
		screen.blit(font.render('FNR Status:', True, myColor), (10, 40))
		screen.blit(font.render('Throttle Status:', True, myColor), (10, 70))
		screen.blit(font.render('Steering Status:', True, myColor), (10, 100))

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
		#Throttle Rectangle
		rect = pygame.draw.rect(screen, (0,204,0), (250, 75, 250, 15), 2)	
		rect = pygame.draw.rect(screen, (0,204,0), (250, 75, publishedThrottle*100*2.5/40, 15), 0)	

		#Steering Rectangle
		rect = pygame.draw.rect(screen, (0,204,0), (250, 75, 250, 15), 2)	
		rect = pygame.draw.rect(screen, (0,204,0), (250, 75, publishedThrottle*100*2.5/40, 15), 0)	



		
		#if radius > 0:
		#circle = pygame.draw.circle(screen, (255,0,0), (500+50+radius,500+150), radius, 0)
		#screen.blit(font.render(str(degrees), True, myColor), (175, 150))


		#screen.blit(font.render(str(radius), True, myColor), (175, 175))
	
		
		#if degrees > 0:
			#radius = int(65/math.tan(degrees+1))
			#circle = pygame.draw.circle(screen, (255,0,0), (500+50+radius,500+150), radius, 0)
		pygame.display.update()	


		
if __name__  == "__main__":
	print "FNR => UP, DOWN, RSHIFT"
	print "Steering => LEFT, RIGHT"
	print "Throttle => W, S"
	KeyboardCtrls()
#i want to be able to store a value for throttle, FNR, steering
