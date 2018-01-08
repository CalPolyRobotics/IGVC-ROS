import rospy
import serial

from collections import deque

from BoardCommsPub import PUB_CALLBACK_LUT
from MESSAGES import *
from Message import Message
from Packet import Packet


GET_MESSAGES = [
    Message(msg_type=MTYPE['get_status']),
    Message(msg_type=MTYPE['get_fnr']),
    Message(msg_type=MTYPE['get_speed']),
    Message(msg_type=MTYPE['get_battery']),
    Message(msg_type=MTYPE['get_power'])
]

class CommsHandler(object):
    """
    Communication manager
    Manages information exchange with the given port

    port - the name of the port to connect to over serial
    baud - the baud rate to communicate over serial
    """

    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.mqueue = deque()
        self.seq_num = 0
        self.gt_idx = 0

    def run(self):
        """
        Board Comms process that continually runs until killed to manage communication
        to and from the control board
        """

        rate = rospy.Rate(1000) # 1000hz

        while not rospy.is_shutdown():
            if len(self.mqueue) > 0:
                ret_pack = self.send_message(self.mqueue.pop())

                if ret_pack is not None and ret_pack.get_type() in PUB_CALLBACK_LUT:
                    PUB_CALLBACK_LUT[ret_pack.get_type()](ret_pack.get_data())
            else:
                self.enqueue_message(GET_MESSAGES[self.gt_idx])
                self.gt_idx = (self.gt_idx + 1) % len(GET_MESSAGES)

            rate.sleep()

        rospy.spin()

    def enqueue_message(self, message):
        """
        Appends a message to the comms handler message queue
        """
        self.mqueue.appendleft(message)

    def send_message(self, message):
        """
        Sends the packet to the board
        packet Packet - packet to send to the board
        """
        pack = Packet(message, self.seq_num).to_bytearray()
        self.seq_num = (self.seq_num + 1) % 256

        self.ser.write(pack)

        # Return packet will be of the same message type + 1
        return self.read_packet(pack[MSG_TYP_IDX] + 1)

    def read_packet(self, msg_type):
        """
        Reads the packet response from the golf cart board
        return Packet - the response from the golf cart
        """
        # Read in header
        header = bytearray(self.ser.read(HEAD_SIZE))

        # Check the message for errors
        if not (len(header) == HEAD_SIZE and
                header[STRT_BYT_1_IDX] == STRT_BYT_1 and
                header[STRT_BYT_2_IDX] == STRT_BYT_2 and
                msg_type == header[MSG_TYP_IDX]):

            # Clear Buffer
            self.ser.flushInput()
            return None

        # CRC is at the end of the packet
        data_len = header[PKT_LEN_IDX] - HEAD_SIZE - 1

        # Append each piece of data to a byte array
        data = bytearray(self.ser.read(data_len))

        #TODO Check CRC

        if not (len(data) == data_len and
                len(data) >= MSG_INFO[header[MSG_TYP_IDX]]["min_length"] and
                len(data) <= MSG_INFO[header[MSG_TYP_IDX]]["max_length"]):

            # Clear Buffer
            self.ser.flushInput()
            return None

        crc = self.ser.read(1)
        # TODO chec crc is valid?

        return Packet(Message(header[MSG_TYP_IDX], data), header[SEQ_NUM_IDX], crc)
