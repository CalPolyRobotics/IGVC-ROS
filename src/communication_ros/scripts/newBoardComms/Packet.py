"""
Packet

This module contains multiple Packet objects for communicating with the golf
cart
"""

from MESSAGES import * 

def format_data(data):
    """
    Returns a hex string of the data array
    """
    return '[' + ', '.join(['0x%X']*len(data)) % tuple(data) + ']'

class Packet(object):
    """
    An object representing packets for board communication
    crc byte - Checksum for message, unused for outgoing packets
    msg_type byte - Value for the specific message type
    seq_num byte - The current sequence number for the packet being sent,
        unused for outgoing packets
    data bytearray - Data being sent in the message, expect empty array
        for no data
    """
    def __init__(self, crc, msg_type, seq_num, data):
        self.msg_type = msg_type
        self.seq_num = seq_num
        self.crc = crc
        self.data = data
        self.length = HEAD_SIZE + len(data)

    def build_bytearray(self):
        """
        Builds a bytearray message for the current packet
        """
        arr = bytearray([STRT_BYT_1, STRT_BYT_2, self.crc, self.msg_type, self.seq_num,
                         self.length])
        for dat in self.data:
            arr.append(dat)

        return arr


    def set_sequence_num(self, seq_num):
        """
        Sets the sequence number of the packet
        """
        self.seq_num = seq_num

    def calc_crc(self):
        """
        Calculates the 8-bit crc for the message
        TODO
        """
        self.crc = 0xCC

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
