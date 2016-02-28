#!/usr/bin/env python

import rospy
import serial

from std_msgs.msg import UInt8, UInt16


start_byte_1 = 0xF0
start_byte_2 = 0x5A
#sequence number counts the messages up to 255, wrap around to 0
s_number = 0 #dictionary {message_type: fcn}
#tracks the message types
message_type = [i for i in range(0x00,0x17)]
send_message_type = 0
#error checking, to be implemented later
CRC8 = 0
header_packet_length = 6
data_msg = 0

def callbackSFNR(data):
	rospy.loginfo(rospy.get_name() + "The FNR data is %d ", data.data)
	global s_number
	global message_type
	global send_message_type
	global header_packer_length
	global data_msg
	global ser
	if s_number < 255:
		s_number = s_number + 1
	else:
		s_number = 0
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
	rospy.loginfo(rospy.get_name() + "The throttle data is %d ", data.data)
	if s_number < 255:
		s_number = s_number +  1
	else:
		s_number = 0
	send_message_type = message_type[8]
	packet_length = header_packet_length + 2
	data_msg = data.data
	packet = bytearray([start_byte_1, start_byte_2, CRC8, send_message_type, s_number, packet_length, (data_msg >> 8) & 0xFF, data_msg & 0xFF])
	print(''.join(format(x, '02x') for x in packet))
	ser.write(packet)

def callbackSSteering(data):
	global s_number
	global message_type
	global send_message_type
	global header_packet_length
	global data_msg
	global ser
	rospy.loginfo(rospy.get_name() + "The steering data is %d ", data.data)
	if s_number < 256:
		s_number = s_number + 1
	else:
		s_number = 0
	send_message_type = message_type[14]
	packet_length = header_packet_length + 2
	data_msg = data.data
	packet = bytearray([start_byte_1, start_byte_2, CRC8, send_message_type, s_number, packet_length, (data_msg >> 8) & 0xFF, data_msg & 0xFF])
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
	#will be able to send as serial message
	ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)

	listener()

