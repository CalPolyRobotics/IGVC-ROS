"""
Packet

This module contains multiple Packet objects for communicating with the golf
cart
"""

import serial

STRT_BYT_1 = 0xF0  # Start Byte 1
STRT_BYT_2 = 0x5A  # Start Byte 2
HEAD_SIZE = 6    # Size of Header

# Indecices of bytes in packet
STRT_BYT_1_IDX = 0
STRT_BYT_2_IDX = 1
CRC_IDX = 2
MSG_TYP_IDX = 3
SEQ_NUM_IDX = 4
PKT_LEN_IDX = 5

MSG_INFO = {
    0x00 : {"name": "Get Status", "length": 0},
    0x01 : {"name": "Get Status Response", "length": 0},
    0x02 : {"name": "Get Sonar 1", "length": 1},
    0x03 : {"name": "Get Sonar 1 Response", "length": 0},
    0x04 : {"name": "Get Sonar All", "length": 0},
    0x05 : {"name": "Get Sonar All Response", "length": 12},
    0x06 : {"name": "Set FNR", "length": 1},
    0x07 : {"name": "Set FNR Response", "length": 0},
    0x08 : {"name": "Get FNR", "length": 0},
    0x09 : {"name": "Get FNR Response", "length": 1},
    0x0A : {"name": "Set Throttle", "length": 2},
    0x0B : {"name": "Set Throttle Response", "length": 0},
    0x0C : {"name": "Set Speed", "length": 2},
    0x0D : {"name": "Set Speed Response", "length": 0},
    0x0E : {"name": "Get Speed", "length": 0},
    0x0F : {"name": "Get Speed Response", "length": 2},
    0x10 : {"name": "Set Steering", "length": 2},
    0x11 : {"name": "Set Steering Response", "length": 0},
    0x12 : {"name": "Get Steering", "length": 0},
    0x13 : {"name": "Get Steering Response", "length": 2},
    0x14 : {"name": "Set Lights", "length": 2},
    0x15 : {"name": "Set Lights Response", "length": 0},
    0x16 : {"name": "Get Battery", "length": 0},
    0x17 : {"name": "Get Battery Response", "length": 4},
    0x18 : {"name": "Get Power", "length": 0},
    0x19 : {"name": "Get Power Response", "length": 16},
    0x1A : {"name": "Stop", "length": 0},
    0x1B : {"name": "Stop Response", "length": 1}
}

def format_data(data):
    """
    Returns a hex string of the data array
    """
    return '[' + ', '.join(['0x%X']*len(data)) % tuple(data) + ']'

class CommsHandler(object):
    """
    Communication manager
    Manages information exchange with the given port
    """

    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.seq_num = 0

    def send_message(self, out_packet):
        """
        Sends the OutgoingPacket to the board
        out_packet OutgoingPacket - packet to send to the port
        """
        mess = out_packet.build_bytearray()

        # Update Sequenc Number
        mess[SEQ_NUM_IDX] = self.seq_num
        self.seq_num = (self.seq_num + 1) % 256

        # Write Message and Return Response
        self.ser.write(mess)
        return self.read_message(mess[MSG_TYP_IDX] + 1)

    def read_message(self, msg_type):
        """
        Reads the message response from the golf cart board
        return IncomingPacket
        """
        # Read in header
        header = bytearray(self.ser.read(HEAD_SIZE))

        # Check the message for errors
        if (len(header) > 0 and header[STRT_BYT_1_IDX] == STRT_BYT_1 and
                header[STRT_BYT_2_IDX] == STRT_BYT_2 and
                msg_type == header[MSG_TYP_IDX]):

            data_len = header[PKT_LEN_IDX] - HEAD_SIZE

            # Append each piece of data to a byte array
            data = bytearray(self.ser.read(data_len))

            if (len(data) == data_len and
                    len(data) == MSG_INFO[header[MSG_TYP_IDX]]["length"]):

                return Packet(header[CRC_IDX], header[MSG_TYP_IDX], header[SEQ_NUM_IDX], data)

        # Return None and clear buffer if Incoming Packet is incorrect
        self.ser.flushInput()
        return None



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
