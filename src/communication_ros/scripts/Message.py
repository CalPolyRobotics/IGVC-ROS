"""
Message

Contains information regarding the data and message type of
communication between the golf cart and computer

"""

from MESSAGES import MSG_INFO

def format_data(data):
    """
    Returns a hex string of the data array
    """
    return '[' + ', '.join(['0x%X']*len(data)) % tuple(data) + ']'

class Message(object):
    """
    An object representing messages sent between the golf cart and computer
    msg_type byte - the specific message type being sent as documented online
    data bytearray - the data being sent in the message
    """
    def __init__(self, msg_type, data=None):
        self.msg_type = msg_type
        if data is None:
            self.data = bytearray()
        else:
            self.data = data

    def get_data(self):
        """
        return bytearry - the data of the message
        """
        return self.data

    def get_type(self):
        """
        return byte - the message type of the message
        """
        return self.msg_type

    def __str__(self):
        """
        Returns a string representation of the message
        """
        return "Message:\n\tType: {}\n\tData {}".format(
            MSG_INFO[self.msg_type]["name"], format_data(self.data))
