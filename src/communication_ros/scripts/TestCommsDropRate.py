#!/usr/bin/env python

import rospy
import serial
import sys
import binascii

from BoardComms import read_msg
from CONFIG import *

from std_msgs.msg import UInt8, UInt16, UInt16MultiArray, UInt8MultiArray

def init_test():
   """ Initialize the success rate test """
   global seq, packet_queue, tests_tot, tests_failed

   seq = 0
   tests_tot = 0
   tests_failed = 0

   rospy.init_node('TestCommsThroughput', anonymous=True)

def build_msg(num, data):
   """ Build a ByteArray Packet """
   global seq

   CRC_8 = 0
   packet = [start_1, start_2, CRC_8, num, seq, DATA_CODES[DATA_TYPES[num]][1] + header_size]

   # Append each byte of the data
   for i in range(DATA_CODES[DATA_TYPES[num]][1] - 1, -1, -1):
      packet.append((data >> (8 *i)) & 0xFF)

   return bytearray(packet)

def run_test():
   """ Run the testing process """
   global seq, packet_queue, ser, tests_tot, tests_failed

   init_test()

   rate = rospy.Rate(1000) # 1000hz

   while( tests_tot < 5000 ):
      tests_tot += 1
      # Write Status Packet
      packet = build_msg( 0x00, None )
      ser.write( packet )
      data = read_msg( ser, packet )

      # Failed Message Return None
      if data == None:
         tests_failed += 1
         ser.flushInput()

      seq = (seq + 1) % 256
      rate.sleep()

   print "Pass Message Rate:", 100 * (1 - tests_failed/float(tests_tot))

if __name__ == '__main__':
   global ser

	#will be able to send as serial message
   args = sys.argv
   if len(args) > 1:
      ser = serial.Serial('/dev/' + args[1], 115200, timeout=0)
   else:
      ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)

   run_test()
