#!/usr/bin/env python

import rospy
import serial
import sys

from collections import deque 

from CRC8 import *
from config import *
from BoardCommsPub import *
from BoardCommsSub import *

from std_msgs.msg import UInt8, UInt16, UInt16MultiArray, UInt8MultiArray 

"Initialize values and setup Node"

def init():
   global seq
   global packet_queue
   global ser

   seq = 0
   packet_queue = deque()
   init_crc8()

   rospy.init_node('BoardComms', anonymous=True)

   # Initialize subscribers
   initSubscribers()

"""
Makes and enqueues a new message to the queue
@params: num = message type, seq = sequence number, data = data assoicated with the message
"""
def enqueue_msg(num, data):
   global seq
   global packet_queue

   #CRC_8 = crc8(msg, 3)
   CRC_8 = 0
   packet = [start_1, start_2, CRC_8, num, seq, DATA_CODES[DATA_TYPES[num]][1] + header_size]
   #print(''.join(format(x, '02x') for x in packet))
   
   # Append each byte of the data
   for i in range(DATA_CODES[DATA_TYPES[num]][1] - 1, -1, -1):
      packet.append((data >> (8 *i)) & 0xFF)

   msg = bytearray(packet)

   packet_queue.appendleft(msg)

def read_msg(serial, packet):
   tmp = serial.readline()
   tmp = tmp.encode("hex")
   
   if (tmp is not None) and (tmp[0:2] == 'f0') and (tmp[2:4] == '5a') and (len(tmp) - header_size * 2== DATA_CODES[DATA_TYPES[packet[3] + 1]][1] * 2):
      data = tmp[2*header_size:] 

   else:
      data = None 
      print("Data is None")

   return data 


def run_process():
   global seq
   global packet_queue
   global ser

   init()

   initSubscribers()

   rate = rospy.Rate(10) # 10hz

   pub_ind = 0

   while not rospy.is_shutdown():
      # Clear packets in packet_queue
      if len(packet_queue) > 0:
         packet = packet_queue.pop()

         ser.write(packet)
         rospy.sleep(.01)
         data = read_msg(ser, packet)

         # Will Not Work For Info getting messages
         if not data == None:
            publisher_callbacks[packet[3]](data)

         # Increase seq -> max is 255
         seq = (seq + 1) % 256

      # Append a publish packet to the packet_queue 
      else:
         enqueue_msg(msg_type[pub_ind], None)
         pub_ind = (pub_ind + 1) % len(msg_type)
      
      rate.sleep()

   rospy.spin()

"Funtions for the subscriber functionality of BoardComms"

def initSubscribers():
   rospy.Subscriber('Set_FNR', UInt8, callbackSFNR)
   rospy.Subscriber('Set_Throttle', UInt16, callbackSThrottle)
   rospy.Subscriber('Set_Steering', UInt16, callbackSSteering)
   rospy.Subscriber('Set_Speed', UInt16, callbackSSpeed)
   rospy.Subscriber('Set_Lights', UInt16, callbackSLights)
   rospy.Subscriber('Stop', UInt8, callbackSStop) # Data is irrelevant

def callbackSFNR(data):
   enqueue_msg(DATA_CODES["SET_FNR"][0], data.data)

def callbackSSteering(data):
   enqueue_msg(DATA_CODES["SET_STEERING"][0], data.data)

def callbackSThrottle(data):
   enqueue_msg(DATA_CODES["SET_THROTTLE"][0], data.data)

def callbackSSpeed(data):
   enqueue_msg(DATA_CODES["SET_STEERING"][0], data.data)

def callbackSLights(data):
   enqueue_msg(DATA_CODES["SET_LIGHTS"][0], data.data)

def callbackSStop(data):
   enqueue_msg(DATA_CODES["STOP"][0], data.data)

"-----------------End subscriber methods--------------"


if __name__ == '__main__':
   global ser
	#will be able to send as serial message
   args = sys.argv
   if len(args) > 1:
      ser = serial.Serial('/dev/' + args[1], 115200, timeout=0)
   else:
      ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)

   run_process()
