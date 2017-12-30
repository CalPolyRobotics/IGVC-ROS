"""
Packet

This module contains multiple Packet objects for communicating with the golf
cart
"""

from MESSAGES import * 
from Message import Message

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
        self.length = HEAD_SIZE + len(self.data)
        self.seq_num = seq_num

        if crc is None:
            self.crc = self.calc_crc
        else:
            self.crc = self.calc_crc()

    def to_bytearray(self):
        """
        Builds a bytearray message for the current packet
        TODO: CRC goes to end of message
        """
        arr = bytearray([STRT_BYT_1, STRT_BYT_2, self.crc, self.msg_type,
                         self.seq_num, self.length])

        for dat in self.data:
            arr.append(dat)

        return arr

    def calc_crc(self):
        """
        Calculates the 8-bit crc for the message
        TODO
        """
        return 0xCC

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
        return "Packet:\n\tType: {}\n\tCRC:{}\n\tSeq Num: {}\n\tData {}".format(
            MSG_INFO[self.msg_type]["name"], self.crc, self.seq_num, format_data(self.data))
