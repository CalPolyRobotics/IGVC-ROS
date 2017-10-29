"""
Unit Tests for Packet Object
"""
import serial
from Packet import CommsHandler, Packet, STRT_BYT_1, STRT_BYT_2
import time

#CH = CommsHandler('/dev/ttyACM0', 115200)

def get_status(comms_handler):
    """
    Docstring
    """
    pack = Packet(crc=0, msg_type=0x00, seq_num=0x00, data=[])
    return comms_handler.send_message(pack)

def get_fnr(comms_handler):
    """
    Docstring
    """
    pack = Packet(crc=0, msg_type=0x08, seq_num=0x00, data=[])
    return comms_handler.send_message(pack)

def get_speed(comms_handler):
    """
    Docstring
    """
    pack = Packet(crc=0, msg_type=0x0E, seq_num=0x00, data=[])
    return comms_handler.send_message(pack)

def get_battery(comms_handler):
    """
    Docstring
    """
    pack = Packet(crc=0, msg_type=0x16, seq_num=0x00, data=[])
    return comms_handler.send_message(pack)

def get_power(comms_handler):
    """
    Docstring
    """
    pack = Packet(crc=0, msg_type=0x18, seq_num=0x00, data=[])
    return comms_handler.send_message(pack)



def run_test(name, func):
    print "Running " + name + " Test"
    if func():
        print "\tPass"
    else:
        print "\tFail"

def test_build_bytearray():
    """
    Unit Test for Packet.build_bytearray
    """
    pack = Packet(crc=0xCC, msg_type=0x00, seq_num=0x00, data=[])
    exp = bytearray([STRT_BYT_1, STRT_BYT_2, 0xCC, 0x00, 0x00, 6])
    return exp == pack.build_bytearray()

def test_send_message():
    """
    Unit test for CommsHandler.send_message()
    """
    CH = CommsHandler('/dev/ttyACM0', 115200)

    pack = Packet(crc=0xCC, msg_type=0x00, seq_num=0x00, data=[])
    exp = bytearray([STRT_BYT_1, STRT_BYT_2, 0xCC, 0x01, 0x00, 6])
    resp = CH.send_message(pack)
    print resp.data
    
    if resp == None:
        return False

    resp = resp.build_bytearray()
    return resp == exp

run_test("Packet - build_bytearray", test_build_bytearray)
run_test("CommsHandler - send_message", test_send_message)

CH = CommsHandler('/dev/ttyACM0', 115200)
print get_status(CH)
print get_fnr(CH)
print get_speed(CH)
print get_battery(CH)
print get_power(CH)
