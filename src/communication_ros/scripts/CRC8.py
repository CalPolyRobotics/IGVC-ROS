#/usr/bin/env python

global crc8_table
def init_crc8():
   global crc8_table
   crc8_table = []
   CRC_8 = 0
   for i in range(256):
      crc = i
      for i in range(8):
         crc = (crc << 1)
         if crc & 0x80:
            crc ^= 0x07
         else:
            crc ^= 0
         crc8_table.append(crc & 0xFF)

def crc8(crc, m):
   crc = crc8_table[crc ^ m]
   crc &= 0xFF
   return crc
