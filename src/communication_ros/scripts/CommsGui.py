#!/usr/bin/env python
import rospy, sys, pygame
from std_msgs.msg import UInt8, UInt16, Float32MultiArray

# general constants
FRAME_RATE = 30
SCREEN_SIZE = [800, 600]
COLOR_GREY = [235, 235, 235]
COLOR_BLACK = [0, 0, 0]
TEXT_LOCATION_START = [15, 15]
TEXT_LOCATION_BUFFER = 50

class GUI():
    def __init__(self, screen):
        self.labelFont = pygame.font.SysFont("Ubuntu", 12)
        self.textFont = pygame.font.SysFont("Ubuntu", 12)
        self.localBufferX = 0
        self.localBufferY = 0
        screen.fill(COLOR_GREY)
        pass

    def printText(self, screen, label, text):
        label = self.labelFont.render(str(label), 5, COLOR_BLACK)
        text = self.textFont.render(str(text), 5, COLOR_BLACK)
        screen.blit(label, (TEXT_LOCATION_START[0], TEXT_LOCATION_START[1]+self.localBufferY) )
        screen.blit(text, (TEXT_LOCATION_START[0], TEXT_LOCATION_START[1]+15+self.localBufferY) )
        self.localBufferY += TEXT_LOCATION_BUFFER
    
    def resetBuffers(self, screen):
        self.localBufferX = 0
        self.localBufferY = 0

    def loop(self, screen):
        clock = pygame.time.Clock()

        while True:
            delta = clock.tick( FRAME_RATE )

            # handle input events, including sys exit
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return #end loop & quit

            GUI.printText(self, screen, "hello", "world")
            GUI.printText(self, screen, "hi", "there")
            GUI.printText(self, screen, "Boo", "Yeah")
            GUI.resetBuffers(self, screen)

            # update display
            pygame.display.update()


    def quit(self):
        pass

def main():
    pygame.init()
    screen = pygame.display.set_mode( (SCREEN_SIZE[0], SCREEN_SIZE[1]) )
    pygame.display.set_caption( 'IGVC (Get) Values' )
    gui = GUI(screen)
    gui.loop( screen )
    gui.quit()

    pygame.quit()

if __name__ == '__main__':
    main()
