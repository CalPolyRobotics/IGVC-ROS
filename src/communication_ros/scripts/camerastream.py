#!/usr/bin/env python

import pygame, rospy, math
import pygame.camera
from pygame.locals import *
from std_msgs.msg import UInt8, UInt16, Float32MultiArray
from numpy import interp

DEVICE = '/dev/video0'
SIZE = (960, 720)
#640, 380

FILENAME = 'capture.png'
#Constants for Drawing
yellow = (200,200,0)
x = 300
y = 200
width = 200
height = 300
startXPos = 150
endXPos = 350
yPos = 250

publishedSteering = 32767
def CallbackSteering(data):
    global publishedSteering
    publishedSteering = data.data


def drawStaticParking(screen):
    global publishedSteering
    pygame.draw.aaline(screen, yellow, (320, 400), (200, 860), 10)
    pygame.draw.aaline(screen, yellow, (640, 400), (760, 860), 10)
    for i in range(0, 4):
        pygame.draw.line(screen, yellow, (640, 400+75*i), (560, 400+75*i), 5)
        pygame.draw.line(screen, yellow, (320, 400+75*i), (400, 400+75*i), 5)

    """
    if publishedSteering < 32000:
        drawAngle = interp((publishedSteering), [0,65534], [600, 200])
        pygame.draw.arc(screen, yellow, (320,400,400,drawAngle), 30, 32, 5)
        pygame.draw.arc(screen, yellow, (640,400,400,drawAngle), 30, 32, 5)

    elif publishedSteering > 33000:
        drawAngle = interp((publishedSteering), [0,65534], [500, 600])
        pygame.draw.arc(screen, yellow, (320,400,400,drawAngle), 40, 41, 5)
        pygame.draw.arc(screen, yellow, (640,400,400,drawAngle), 40, 41, 5)
    else:
        pygame.draw.line(screen, yellow, (320, 400), (320, 860), 5)
        pygame.draw.line(screen, yellow, (640, 400), (640, 860), 5)
        for i in range(0, 4):
            pygame.draw.line(screen, yellow, (640, 400+75*i), (560, 400+75*i), 5)
            pygame.draw.line(screen, yellow, (320, 400+75*i), (400, 400+75*i), 5)
    """



def camstream():
    #ROS Subscription Initialization
    rospy.init_node('camstream', anonymous=True)
    rospy.Subscriber("Set_Steering", UInt16, CallbackSteering)
    #Camera & Pygame Screen Initialization
    pygame.init()
    font = pygame.font.SysFont('dejavusansmono', 15)
    pygame.camera.init()
    display = pygame.display.set_mode(SIZE, 0)
    camera = pygame.camera.Camera(DEVICE, SIZE)
    camera.start()
    screen = pygame.surface.Surface(SIZE, 0, display)
    capture = True
    while capture:
        screen = camera.get_image(screen)
        #Draw Static Lines
        drawStaticParking(screen)
        screen.blit(font.render(str(publishedSteering), True, (255,255,0)), (175, 175))
        display.blit(screen, (0,0))
        pygame.display.flip()
        pressed = pygame.key.get_pressed()
        for event in pygame.event.get():
            if event.type == QUIT:
                capture = False
            if pressed[pygame.K_q]:
                camera.stop()
                pygame.quit()
    camera.stop()
    pygame.quit()
    return

if __name__ == '__main__':
    camstream()
