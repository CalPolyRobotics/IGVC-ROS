#!/usr/bin/env python
import rospy, sys, os, datetime, pygame
from std_msgs.msg import UInt8, UInt8MultiArray, UInt16, UInt16MultiArray, Float32MultiArray

# general constants
FRAME_RATE = 30
SCREEN_SIZE = [400, 450]
COLOR_GREY = [235, 235, 235]
COLOR_BLACK = [0, 0, 0]
TEXT_LOCATION_START = [15, 15]
TEXT_LOCATION_BUFFER = 55

info = {
    "FNR"    : {"name" : "FNR:", "value" : ""},
    "SPDL"   : {"name" : "Speed Left (m/s):", "value": ""},
    "SPDR"   : {"name" : "Speed Right (m/s):", "value": ""},
    "STR"    : {"name" : "Steering:", "value": ""},
    "PDL"    : {"name" : "Pedal:", "value": ""},
    "PWVB"   : {"name" : "Battery (V):", "value": ""},
    "PWAB"   : {"name" : "Battery (A):", "value": ""},
    "PWV3"   : {"name" : "3.3V Rail (V):", "value": ""},
    "PWA3"   : {"name" : "3.3V Rail (A):", "value": ""},
    "PWV5"   : {"name" : "5V Rail (V):", "value": ""},
    "PWA5"   : {"name" : "5V Rail (A):", "value": ""},
    "PWV12"  : {"name" : "12V Rail (V):", "value": ""},
    "PWA12"  : {"name" : "12V Rail (A):", "value": ""}
}

AVG_VAL = 16
speed_bufl = [0 for x in range(AVG_VAL)]
speed_bufr = [0 for x in range(AVG_VAL)]
speed_idx = 0

steer_buf = [0 for x in range(AVG_VAL)]
steer_idx = 0

pedal_buf = [0 for x in range(AVG_VAL)]
pedal_idx = 0


def init_subscribers():
    """
    Initialize the subscribers for the hud
    """
    rospy.Subscriber('Get_FNR', UInt8, fnr_callback)
    rospy.Subscriber('Get_Speed', UInt16MultiArray, speed_callback)
    rospy.Subscriber('Get_Steering', UInt16, steer_callback)
    rospy.Subscriber('Get_Pedal', UInt16, pedal_callback)
    rospy.Subscriber('Get_Power', UInt16MultiArray, power_callback)

def fnr_callback(data):
    """
    Sets FNR value in global info file
    """
    global info
    info["FNR"]["value"] = data.data

def speed_callback(data):
    """
    Averages speed values for left and right wheels
    Stores averaged values in info
    """
    global info, speed_bufl, speed_bufr, speed_idx
    speed_bufl[speed_idx] = data.data[0]
    speed_bufr[speed_idx] = data.data[1]
    speed_idx = (speed_idx + 1) % AVG_VAL

    if speed_idx == 0:
        avgl = 0
        avgr = 0
        for i in range(0, AVG_VAL):
            avgl += speed_bufl[i]
            avgr += speed_bufr[i]
        info["SPDL"]["value"] = avgl/AVG_VAL/1000.0
        info["SPDR"]["value"] = avgr/AVG_VAL/1000.0

def steer_callback(data):
    """
    Averages steer values and stores averaged values in info
    """
    global info, steer_buf, steer_idx
    steer_buf[steer_idx] = data.data
    steer_idx = (steer_idx + 1) % AVG_VAL

    if steer_idx == 0:
        avg = 0
        for i in range(0, AVG_VAL):
            avg += steer_buf[i]
        info["STR"]["value"] = avg/AVG_VAL

def pedal_callback(data):
    """
    Averages pedal values and stores averaged values in info
    """
    global info, pedal_buf, pedal_idx
    pedal_buf[pedal_idx] = data.data
    pedal_idx = (pedal_idx + 1) % AVG_VAL

    if pedal_idx == 0:
        avg = 0
        for i in range(0, AVG_VAL):
            avg += pedal_buf[i]
        info["PDL"]["value"] = avg/AVG_VAL

def power_callback(data):
    """
    Averages pedal values and stores averaged values in info
    """
    global info
    info["PWVB"]["value"] = data.data[0]
    info["PWAB"]["value"] = data.data[1]
    info["PWV3"]["value"] = data.data[2] 
    info["PWA3"]["value"] = data.data[3]
    info["PWV5"]["value"] = data.data[4]
    info["PWA5"]["value"] = data.data[5]
    info["PWV12"]["value"] = data.data[6]
    info["PWA12"]["value"] = data.data[7]

class HUD():
    def __init__(self, screen):
        self.backgroundImage = pygame.image.load(os.path.abspath("resources/LTSpice.png"))
        self.labelFont = pygame.font.SysFont('dejavusansmono', 17)
        self.labelFont.set_underline(True)
        self.textFont = pygame.font.SysFont('dejavusansmono', 17)
        self.localBufferX = 0
        self.localBufferY = 0
        screen.fill(COLOR_GREY)
        pass

    def printText(self, screen, label, text):
        label = self.labelFont.render(str(label), 5, COLOR_BLACK)
        text = "No data detected" if len(str(text)) == 0 else text

        text = self.textFont.render(str(text), 5, COLOR_BLACK)
        screen.blit(label, (TEXT_LOCATION_START[0]+self.localBufferX, TEXT_LOCATION_START[1]+self.localBufferY) )
        screen.blit(text, (TEXT_LOCATION_START[0]+self.localBufferX, TEXT_LOCATION_START[1]+15+self.localBufferY) )
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

            HUD.printText(self, screen, info["FNR"  ]["name"], info["FNR"  ]["value"])
            HUD.printText(self, screen, info["SPDL" ]["name"], info["SPDL" ]["value"])
            HUD.printText(self, screen, info["STR"  ]["name"], info["STR"  ]["value"])
            HUD.printText(self, screen, info["PWV12"]["name"], info["PWV12"]["value"])
            HUD.printText(self, screen, info["PWV5" ]["name"], info["PWV5" ]["value"])
            HUD.printText(self, screen, info["PWV3" ]["name"], info["PWV3" ]["value"])
            HUD.printText(self, screen, info["PWVB" ]["name"], info["PWVB" ]["value"])
            HUD.resetBuffers(self, screen)
            self.localBufferX += SCREEN_SIZE[0]/2
            HUD.printText(self, screen, info["PDL"  ]["name"], info["PDL"  ]["value"])
            HUD.printText(self, screen, info["SPDR" ]["name"], info["SPDR" ]["value"])
            HUD.printText(self, screen, "OS Time", datetime.datetime.time(datetime.datetime.now()))
            HUD.printText(self, screen, info["PWA12"]["name"], info["PWA12"]["value"])
            HUD.printText(self, screen, info["PWA5" ]["name"], info["PWA5" ]["value"])
            HUD.printText(self, screen, info["PWA3" ]["name"], info["PWA3" ]["value"])
            HUD.printText(self, screen, info["PWAB" ]["name"], info["PWAB" ]["value"])

            # update display
            HUD.resetBuffers(self, screen)
            pygame.display.update()
            #screen.fill(COLOR_GREY)
            screen.blit(self.backgroundImage, (0,0))


    def quit(self):
        pass

def main():
    rospy.init_node('BoardComms', anonymous=True)
    init_subscribers()
    pygame.init()
    screen = pygame.display.set_mode( (SCREEN_SIZE[0], SCREEN_SIZE[1]) )
    pygame.display.set_caption( "Cal Poly IGVC - HUD" )
    hud = HUD(screen)
    hud.loop( screen )
    hud.quit()

    pygame.quit()

if __name__ == '__main__':
    main()
