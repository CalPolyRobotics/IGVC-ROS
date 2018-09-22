"""
Packet

This module contains multiple Packet objects for communicating with the golf
cart
"""

from MESSAGES import * 
from Message import Message
from CRC8 import crc8

def format_data(data):
    """
    Returns a hex string of the data array
    """
    return '[' + ', '.join(['0x%X']*len(data)) % tuple(data) + ']'

class Packet(object):
    """
    An object representing packets for board communication
    message - The message being sent in the packet
    seq_num byte - The current sequence number for the packet being sent,
        unused for outgoing packets
    crc byte - Checksum for message, unused for outgoing packets
    """
    def __init__(self, message, seq_num, crc=None):
        self.msg_type = message.get_type()
        self.data = message.get_data()

        # CRC is of length 1
        self.length = HEAD_SIZE + len(self.data) + 1
        self.seq_num = seq_num

        if crc is None:
            self.crc = crc8(self.to_bytearray(withCrc=0))
        else:
            self.crc = crc

    def to_bytearray(self, withCrc=1):
        """
        Builds a bytearray message for the current packet
        withCrc - int include the CRC in building the bytearray
        """
        arr = bytearray([STRT_BYT_1, STRT_BYT_2, self.msg_type,
                         self.seq_num, self.length])

        for dat in self.data:
            arr.append(dat)

        if withCrc == 1:
            arr.append(self.crc)

        return arr

    def get_type(self):
        """
        Returns the message number of the packet
        """
        return self.msg_type

    def get_data(self):
        """
        Returns the data byte array stored in the object
        """
        return self.data

    def __str__(self):
        """
        Returns a string representation of the packet
        """
        return "Packet:\n\tType: {}\n\tSeq Num: {}\n\tData {}\n\tCRC:{}".format(
            MSG_INFO[self.msg_type]["name"], self.seq_num, format_data(self.data), self.crc)
