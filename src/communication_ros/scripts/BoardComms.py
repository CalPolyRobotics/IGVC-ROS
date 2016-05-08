#!/usr/bin/env python

import rospy
import serial
import sys

from collections import deque 

from CRC8 import *
from config import *

from std_msgs.msg import UInt8, UInt16, UInt8MultiArray 

start_1 = 0xF0
start_2 = 0x5A
h_packet = 6 # Size of Header
queue = deque()
seq = 0
init_crc8()

"""
@params: num = message type, seq = sequence number, data = data assoicated with the message
@return: a serial message for that type
"""
def create_msg(num, data):
   global seq
   #CRC_8 = crc8(msg, 3)
   CRC_8 = 0
   packet = [start_1, start_2, CRC_8, num, seq, DATA_CODES[DATA_TYPES[num]][1] + h_packet]
   
   # Append each byte of the data
   for i in range(DATA_CODES[DATA_TYPES[num]][1] - 1, -1, -1):
      packet.append((data.data >> (8 *i)) & 0xFF)
   msg = bytearray(packet)

   return msg

"""
after reading in a serial message, update the data for tracked values
"""
def update_values(msg):
	msg_type = msg[2]
	DATA_VALUE[DATE_TYPE[msg_type]] = msg_data = msg[len(msg) -1]

def read_msg(serial, packet):
   tmp = serial.readline()
   tmp = tmp.encode("hex")
   
   if (tmp is not None) and (tmp[0:2] == 'f0') and (tmp[2:4] == '5a') and (len(tmp) - h_packet * 2== DATA_CODES[DATA_TYPES[packet[3] + 1]][1] * 2):
      data = tmp[2*h_packet:] 
      array = UInt8MultiArray()
      array.data = []

      #TODO Not receiving the correct odd numbered response on the packet
      for i in range(0, DATA_CODES[DATA_TYPES[packet[3] + 1]][1] * 2, 2):
         num = data[i:i+2]
         array.data.append(int(num, 16))
   else:
      array = None 
      print("Data is None")

   return array 

def callbackSFNR(data):
   global queue
   packet = create_msg(DATA_CODES["SET_FNR"][0], data)
   print(''.join(format(x, '02x') for x in packet))
   queue.appendleft(packet)

def callbackSSteering(data):
   global queue 
   packet = create_msg(DATA_CODES["SET_STEERING"][0], data)
   print(''.join(format(x, '02x') for x in packet))
   queue.apendleft(packet)

def callbackSThrottle(data):
   global queue
   packet = create_msg(DATA_CODES["SET_THROTTLE"][0], data)
   print(''.join(format(x, '02x') for x in packet))
   queue.appendleft(packet)

def callbackSSpeed(data):
   global queue
   packet = create_msg(DATA_CODES["SET_STEERING"][0], data)
   print(''.join(format(x, '02x') for x in packet))
   queue.appendleft(packet)

def callbackSLights(data):
   global queue
   packet = create_msg(DATA_CODES["SET_LIGHTS"][0], data)
   print(''.join(format(x, '02x') for x in packet))
   queue.appendleft(packet)

def callbackSStop(data):
   global queue
   packet = create_msg(DATA_CODES["STOP"][0], data)
   print(''.join(format(x, '02x') for x in packet))
   queue.appendleft(packet)

def listener():
   global seq
   global queue

   rospy.init_node('BoardComms', anonymous=True)

   rospy.Subscriber('Set_FNR', UInt8, callbackSFNR)
   rospy.Subscriber('Set_Throttle', UInt16, callbackSThrottle)
   rospy.Subscriber('Set_Steering', UInt16, callbackSSteering)
   rospy.Subscriber('Set_Speed', UInt16, callbackSSpeed)
   rospy.Subscriber('Set_Lights', UInt16, callbackSLights)
   rospy.Subscriber('Stop', UInt8, callbackSStop) # Data is irrelevant
   
   rate = rospy.Rate(10) # 10hz

   status = rospy.Publisher("Status", UInt8MultiArray, queue_size=1000)
   sonar1 =  rospy.Publisher("Get_Sonar_1", UInt8MultiArray, queue_size=1000)
   sonarAll =  rospy.Publisher("Get_Sonar_All", UInt8MultiArray, queue_size=1000)
   fnr = rospy.Publisher("Get_FNR", UInt8MultiArray, queue_size=1000)
   speed =  rospy.Publisher("Get_Speed", UInt8MultiArray, queue_size=1000)
   steering =  rospy.Publisher("Get_Steering", UInt8MultiArray, queue_size=1000)
   battery = rospy.Publisher("Get_Battery", UInt8MultiArray, queue_size=1000)
   power =  rospy.Publisher("Get_Power", UInt8MultiArray, queue_size=1000)
   # Power should output and array of UInt16s
   #msg_type = [0x00, 0x02, 0x04, 0x08, 0x0E, 0x12, 0x16, 0x18]
   msg_type = [0x12, 0x18]
   pubs = {
     # 0x00 : status,
     # 0x02 : sonar1,
     # 0x04 : sonarAll,
     # 0x08 : fnr,
     # 0x0E : speed,
      0x12 : steering,
     # 0x16 : battery,
      0x18 : power}
    
   pub = 0
   while not rospy.is_shutdown():
      if len(queue) > 0:
         packet = queue.pop()

         # Data is R -> C 
         try: 
            # if Cant access from dictionary, data is C -> R
            tmp =  pubs[packet[3]]
            ser.write(packet)
            rospy.sleep(1)
            data = read_msg(ser, packet)
            if not data == None:
               pubs[packet[3]].publish(data)
            # Append each byte of the data
         # Data is C -> R
         except :
            ser.write(packet)

         if seq == 255:
            seq = 0
         else:
            seq += 1

      # if queue is empty 
      else:
         if pub == len(pubs):
            pub = 0
         queue.appendleft(create_msg(msg_type[pub], None))
         pub += 1
      
      rate.sleep()

   rospy.spin()

if __name__ == '__main__':
   global ser
	#will be able to send as serial message
   args = sys.argv
   if len(args) > 1:
      ser = serial.Serial('/dev/' + args[1], 115200, timeout=0)
   else:
      ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)

   listener()
