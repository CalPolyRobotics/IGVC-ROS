start_1  = 0xF0 # Start Byte 1
start_2  = 0x5A # Start Byte 2
header_size = 6    # Size of Header

# Indecices of bytes in packet
START_BYTE1   = 0
START_BYTE2   = 1
CRC8          = 2
MSG_TYPE      = 3
SEQUENCE_NUM  = 4
PACKET_LEN    = 5

class Packet:
   def __init__(self, msgType, data):
      self.msgType = msgType
      self.data = data

   def buildMessage(self):
      return 0

   def readMessage(ser):
      # Read in header
      header = bytearray(ser.read(header_size))
      print("read_msg header:" + str(binascii.hexlify(header)))
      print("Sequence number from read: " + str(packet[4]))

      # Ensure the first two  bytes are the correct start bytes
      if(len(header) > 0 and header[START_BYTE1] == start_1 and header[START_BYTE2] == start_2 and packet[4] == header[4]):
         data_len = header[PACKET_LEN] - header_size

         # Append each piece of data to a byte array
         data = bytearray(ser.read(data_len))

         if ((len(data) == data_len) and (len(data) == DATA_LENGTH[header[MSG_TYPE]])):
            return data

      ser.flushInput()
      return None

class OutgoingPacket(Packet):
   def __init__(self, msgType, data):
      super(Packet, self).__init__(msgType, data)

   def sendPacket(self, ser):
      ser.write(self.buildMessage()) 
      return read_msg(ser, #Length#)


class IncomingPacket(Packet):
   def __init__(self, msgType, crc, seqNum, data):
      super(Packet, self).__init__(msgType, data)
      self.crc = crc
      self.seqNum = seqNum

   def __repr__(self):
      return ""
   
   def __str__(self):
