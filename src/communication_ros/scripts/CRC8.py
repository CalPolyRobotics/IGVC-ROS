#/usr/bin/env python

made_table = 0

def init_crc8():
    for i in range(256):
        crc = i
        for i in range(8):
            crc = (crc << 1) ^ ((crc & 0x80) ? 0x07 : 0)
        crc8_table[i] = crc & 0xFF
    return crc8_table

def crc8(crc, m):
   crc = crc8_table[crc ^ m]
   crc &= 0xFF
   return crc
