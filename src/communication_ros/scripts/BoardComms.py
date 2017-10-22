#!/usr/bin/env python

import rospy
import serial
import sys
import binascii

from collections import deque


from CRC8 import *
from CONFIG import *
from BoardCommsPub import *
from ConversionMethod import *

from std_msgs.msg import UInt8, UInt16, UInt16MultiArray, UInt8MultiArray

"Initialize values and setup Node"

def init():
   global seq
   global packet_queue
   global ser

   seq = 0
   packet_queue = deque()

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

def read_msg(packet):
   global ser
   # Read in header
   header = bytearray(ser.read(header_size))
   print("read_msg header:" + str(binascii.hexlify(header)))
   print("Sequence number from read: " + str(packet[4]))

   # Ensure the first two  bytes are the correct start bytes
   if(len(header) > 0 and header[START_BYTE1] == start_1 and header[START_BYTE2] == start_2 and packet[4] == header[4]):
      data_len = header[PACKET_LEN] - header_size

      # Append each piece of data to a byte array
      data = bytearray(ser.read(data_len))

      if ((len(data) == data_len) and (len(data) == DATA_LENGTH[header[MSG_TYPE]])):
         return data

   ser.flushInput()
   return None

def run_process():
   global seq
   global packet_queue
   global ser

   init()

   rate = rospy.Rate(1000) # 1000hz

   pub_ind = 0

   while not rospy.is_shutdown():
      # Clear packets in packet_queue
      if len(packet_queue) > 0:
         packet = packet_queue.pop()
         ser.write(packet)
         print("Sequence number: " + str(packet[4]))
         data = read_msg(packet)

         # Will Not Work For Info getting messages
         if not data == None:
            # Try to publish data
            #try:
            print( "0x%0.2X" % packet[3] )
            #print(publisher_callbacks[2])
            publisher_callbacks[packet[3]](data)
         else:
            print "Data Error"

            #except:
               # Message_type is not a publisher
               #print("Cannot Publish Type")

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
   print "FNRLength " + str(len(data))
   enqueue_msg(DATA_CODES["SET_FNR"][0], data.data)

def callbackSSteering(data):
   print "steeringLength " + str(len(data))
   enqueue_msg(DATA_CODES["SET_STEERING"][0], data.data)

def callbackSThrottle(data):
   print "throttleLen " + str(len(data))
   enqueue_msg(DATA_CODES["SET_THROTTLE"][0], data.data)

def callbackSSpeed(data):
   print "speedLen " + str(len(data))
   enqueue_msg(DATA_CODES["SET_STEERING"][0], data.data)

def callbackSLights(data):
   print "lightsLen " + str(len(data))
   enqueue_msg(DATA_CODES["SET_LIGHTS"][0], data.data)

def callbackSStop(data):
   print "stopLen " + str(len(data))
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
