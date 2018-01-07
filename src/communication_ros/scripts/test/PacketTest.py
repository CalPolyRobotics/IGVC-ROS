"""
Unit Tests for Packet Object
"""
from Packet import Packet
from CommsHandler import CommsHandler
from Message import Message
from MESSAGES import MTYPE, HEAD_SIZE, STRT_BYT_1, STRT_BYT_2
from test.Test import bytearrays_equal
#CH = CommsHandler('/dev/ttyACM0', 115200)

def test_to_bytearray():
    """
    Unit Test for Packet.build_bytearray
    """
    data = bytearray([0x01, 0x03, 0xFD, 0xFF])
    pack = Packet(Message(MTYPE['get_power'], data), 0xA5)
    exp = bytearray([STRT_BYT_1, STRT_BYT_2, 0xCC, MTYPE['get_status'], 0xA5,
                     HEAD_SIZE + len(data)])

    for dat in data:
        exp.append(dat)

    return exp == pack.to_bytearray()

def test_send_message():
    """
    Unit test for CommsHandler.send_message()
    """
    CH = CommsHandler('/dev/ttyACM0', 115200)

    pack = Packet(Message(MTYPE['get_status']), 0x00, 0xCC)
    exp = bytearray([STRT_BYT_1, STRT_BYT_2, 0xCC, 0x01, 0x00, 6])
    resp = CH.send_message(pack)
    print resp.data

    if resp is None:
        return False

    resp = resp.to_bytearray()
    return resp == exp

def run_packet_test(test):
    """
    Run all the unit tests for Packet
    """
    test.run_test("Packet - build_bytearray", test_to_bytearray)
    test.run_test("CommsHandler - send_message", test_send_message)
