import rospy

import socket
import sys

class EthClient(object):
    def __init__(self, ip, port, timeout):
        '''init'''
        rospy.loginfo("Connecting to ethernet %s:%d"%(ip, port))

        try:
            self.sock = socket.create_connection((ip, port), timeout)
        except socket.timeout:
            rospy.logfatal("Could not establish connection to %s:%d"%(ip, port))
            sys.exit(1)

    def write(self, pkt):
        '''write'''
        self.sock.sendall(pkt)

    def read(self, size=1):
        '''read'''

        data = None

        try:
            data = self.sock.recv(size)

            #TODO - This probably shouldn't block board comms in the meantime
            #Maybe add timeout
            while len(data) < size:
                data += self.sock.recv(size - len(data))
        except socket.timeout:
            rospy.logerr("Packet read timed out")

        return data

    def flushInput(self):
        '''read'''
        return

    def __del__(self):
        '''override destructor'''
        if hasattr(self, 'sock'):
            self.sock.close()
