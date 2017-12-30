"""
MESSAGES
Configuration of messages and Packets
"""

from enum import Enum

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

"""
MTYPE
Dictionary of Message types

response message types are 1 + the message type
"""
MTYPE = {
    "get_status" : 0x00,
    "get_1_sonar" : 0x02,
    "get_all_sonar_1" : 0x04,
    "set_fnr" : 0x06,
    "get_fnr" : 0x08,
    "set_throttle" : 0x0A,
    "set_speed" : 0x0C,
    "get_speed" : 0x0E,
    "set_steering" : 0x10,
    "get_steering" : 0x12,
    "set_lights" : 0x14,
    "get_battery" : 0x16,
    "get_power" : 0x18,
    "send_stop" : 0x1A,
}
