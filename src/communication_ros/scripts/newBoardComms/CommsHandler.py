from Packet import Packet
import rospy
import serial
from collections import deque
from MESSAGES import *
from BoardCommsPub import PUB_CALLBACK_LUT

GET_MESSAGES = [
    Packet(crc=0, msg_type=0x00, seq_num=0x00, data=[]), #Status
    Packet(crc=0, msg_type=0x08, seq_num=0x00, data=[]), #FNR
    #Packet(crc=0, msg_type=0x0E, seq_num=0x00, data=[]), #Speed
    #Packet(crc=0, msg_type=0x16, seq_num=0x00, data=[]), #Battery
    Packet(crc=0, msg_type=0x18, seq_num=0x00, data=[])  #Power
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
        self.msg_sent = 0
        self.msg_fail = 0

    def run(self):
        """
        Board Comms process that continually runs until killed to manage communication
        to and from the control board
        """

        rate = rospy.Rate(1000) # 1000hz

        while not rospy.is_shutdown():
            if len(self.mqueue) > 0:
                msg = self.send_message(self.mqueue.pop())
                self.msg_sent = self.msg_sent + 1

                if msg is None:
                    self.msg_fail = self.msg_fail + 1
                else:
                    if msg.get_type() in PUB_CALLBACK_LUT:
                        PUB_CALLBACK_LUT[msg.get_type()](msg.get_data())
            else:
                self.queue_packet(GET_MESSAGES[self.gt_idx])
                self.gt_idx = (self.gt_idx + 1) % len(GET_MESSAGES)

            rate.sleep()

        rospy.spin()

    def queue_packet(self, packet):
        """
        Appends a packet to the comms handler message queue
        """
        self.mqueue.appendleft(packet)

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

    def get_num_total_packets(self):
        """
        Return the total number of packets sent
        """
        return self.msg_sent

    def get_num_failed_packets(self):
        """
        Return the total number of packets that failed
        """
        return self.msg_fail
