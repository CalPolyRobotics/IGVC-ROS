#!/usr/bin/env python

import rospy
import serial
import sys
import Queue
from crc8 import *
from config import *

from std_msgs.msg import UInt8, UInt16


queue = Queue.Queue(maxsize=0)
seq = 0
init_crc8()

"""
@params: num = message type, seq = sequence number, data = data assoicated with the message
@return: a serial message for that type
"""
def create_msg(num, data):
    global seq
	start_1 = 0xF0
	start_2 = 0x5A
	h_packet = 6
	CRC_8 = 0
	msg = bytearray([start_1, start_2, CRC_8, DATA_CODES[DATA_TYPE[num]][0], seq, DATA_CODES[DATA_TYPE[num]][1] + h_packet, data])

    CRC_8 = crc8(1, msg)
	msg[2] = CRC_8

    return msg


"""
after reading in a serial message, update the data for tracked values
"""
def update_values(msg):
	msg_type = msg[2]
	DATA_VALUE[DATE_TYPE[msg_type]] = msg_data = msg[len(msg) -1]

def read_msg(serial):
	tmp = serial.read(6)
	msg_type = tmp[3]
	if (tmp is not null) and (tmp[0] == 0xF0) and (tmp[1] == 0x5A):
		data = serial.read(DATA_CODE[DATA_TYPE[msg_type]][1])
		tmp.add(data)
	return tmp

def callbackSFNR(data):
	global queue
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
	packet = create_msg(0x06, data)
	print(''.join(format(x, '02x') for x in packet))
	queue.put(packet)

def callbackSThrottle(data):
	global queue
	packet = create_msg(0x08, data)
	print(''.join(format(x, '02x') for x in packet))
	queue.put(packet)

def callbackSSteering(data):
	global queue
	packet = create_msg(0x10, data)
	print(''.join(format(x, '02x') for x in packet))
	queue.put(packet)

def callbackSSpeed(data):
	global queue
	packet = create_msg(0x0A, data)
	print(''.join(format(x, '02x') for x in packet))
	queue.put(packet)

def callbackSLights(data):
	global queue
	packet = create_msg(0x18, data)
	print(''.join(format(x, '02x') for x in packet))
	queue.put(packet)

def listener():
	rospy.init_node('BoardComms', anonymous=True)

    pubData = rospy.Publisher("data", UInt8, queue_size=1000)

	rospy.Subscriber('Set_FNR', UInt8, callbackSFNR())
	rospy.Subscriber('Set_Throttle', UInt16, callbackSThrottle())
	rospy.Subscriber('Set_Steering', UInt16, callbackSSteering())
	rospy.Subscriber('Set_Speed', UInt16, callbackSSpeed())
	rospy.Subscriber('Set_Lights', UInt8, callbackSLights())
	#will have other subscribers
	rospy.spin()

if __name__ == '__main__':
	#will be able to send as serial message
   args = sys.argv
   if len(args) > 1:
      ser = serial.Serial('/dev/' + args[1], 115200, timeout=0)
   else:
      ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
   listener()

	while rospy_is.shutdown():
		tmp_msg = read_msg(ser):
		listener()
		queue.put(create_msg(data[i], none))
		if seq > 255:
			seq = 0
		else:
			seq+=1
        ser.write(queue.get())
        i+= 1

