#!/usr/bin/env python

import rospy
import serial

from std_msgs.msg import UInt8, UInt16


start_byte_1 = 0xF0
start_byte_2 = 0x5A
s_number = 0
message_type = (0x00, 0x01, 0x02,0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17)
send_message_type = 0
CRC8 = 0
header_packet_length = 6
data_msg = 0

def callbackSFNR(data):
	rospy.loginfo(rospy.get_name() + "The data is %d ", data.data)
	global s_number
	global message_type
	global send_message_type
	global header_packer_length
	global data_msg
	global ser
	s_number = s_number + 1
	send_message_type = message_type[6]
	packet_length = header_packet_length + 1
	data_msg = data.data
	packet = bytearray([start_byte_1, start_byte_2, CRC8, send_message_type, s_number, packet_length, data_msg])
	print(''.join(format(x, '02x') for x in packet))
	ser.write(packet)

def callbackSThrottle(data):
	global s_number
	global message_type
	global send_message_type
	global header_packet_length
	global data_msg
	global ser
	rospy.loginfo(rospy.get_name() + "The data is %d ", data.data)
	s_number = s_number +  1
	send_message_type = message_type[8]
	packet_length = header_packet_length + 2
	data_msg = data.data
	packet = bytearray([start_byte_1, start_byte_2, CRC8, send_message_type, s_number, packet_length, data_msg >> 8, data_msg])
	print(''.join(format(x, '02x') for x in packet))
	ser.write(packet)

def callbackSSteering(data):
	global s_number
	global message_type
	global send_message_type
	global header_packet_length
	global data_msg
	global ser
	rospy.loginfo(rospy.get_name() + "The data is %d ", data.data)
	s_number = s_number + 1
	send_message_type = message_type[14]
	packet_length = header_packet_length + 2
	data_msg = data.data
	packet = bytearray([start_byte_1, start_byte_2, CRC8, send_message_type, s_number, packet_length, data_msg >> 8, data_msg])
	print(''.join(format(x, '02x') for x in packet))
	ser.write(packet)

def listener():
	rospy.init_node('BoardComms', anonymous=True)
	rospy.Subscriber('Set_FNR', UInt8, callbackSFNR)
	rospy.Subscriber('Set_Throttle', UInt16, callbackSThrottle)
	rospy.Subscriber('Set_Steering', UInt16, callbackSSteering)
	#will have other subscribers
	rospy.spin()

if __name__ == '__main__':
	ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0)

	listener()

