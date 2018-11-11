"""
MESSAGES
Configuration of messages and Packets
"""

STRT_BYT_1 = 0xF0  # Start Byte 1
STRT_BYT_2 = 0x5A  # Start Byte 2
HEAD_SIZE = 5    # Size of Header

# Indecices of bytes in packet
STRT_BYT_1_IDX = 0
STRT_BYT_2_IDX = 1
MSG_TYP_IDX = 2
SEQ_NUM_IDX = 3
PKT_LEN_IDX = 4
MAX_DATA_LENGTH = 250

MSG_INFO = [ 
   {"name": "Status",                        "length": 0},
   {"name": "Status Response",               "length": 5},
   {"name": "Get Speed",                     "length": 0},
   {"name": "Get Speed Response",            "length": 4},
   {"name": "Get Steering",                  "length": 0},
   {"name": "Get Steering Response",         "length": 2},
   {"name": "Get Sonar",                     "length": 0},
   {"name": "Get Sonar Response",            "length": 8},
   {"name": "Set Turn Signal",               "length": 1},
   {"name": "Set Turn Signal Response",      "length": 0},
   {"name": "Set Headlights",                "length": 2},
   {"name": "Set Headlights Response",       "length": 0},
   {"name": "Set Misc Lights",               "length": 2},
   {"name": "Set Misc Lights Response",      "length": 0},
   {"name": "Get FNR",                       "length": 0},
   {"name": "Get FNR Response",              "length": 1},
   {"name": "Get Auto-Man Control",          "length": 0},
   {"name": "Get Auto-Man Control Response", "length": 1},
   {"name": "Set FNR",                       "length": 1},
   {"name": "Set FNR Response",              "length": 0},
   {"name": "Set Steering",                  "length": 2},
   {"name": "Set Steering Response",         "length": 0},
   {"name": "Set Brake",                     "length": 2},
   {"name": "Set Brake Response",            "length": 0},
   {"name": "Unused ",                       "length": 0},
   {"name": "Unused Response",               "length": 0},
   {"name": "Set Speed",                     "length": 2},
   {"name": "Set Speed Response",            "length": 0},
   {"name": "Set Leds",                      "length": 2},
   {"name": "Set Leds Response",             "length": 0},
   {"name": "Get Power",                     "length": 0},
   {"name": "Get Power Response",            "length": 16},
   {"name": "Kill",                          "length": 0},
   {"name": "Kill Response",                 "length": 0}
]

"""
MTYPE
Dictionary of Message types

response message types are 1 + the message type
"""
MTYPE = {
    "status"           : 0x00,
    "get_speed"        : 0x02,
    "get_steering"     : 0x04,
    "get_sonar"        : 0x06,
    "set_turn_signal"  : 0x08,
    "set_headlights"   : 0x0A,
    "set_misc_lights"  : 0x0C,
    "get_fnr"          : 0x0E,
    "get_aman_control" : 0x10,
    "set_fnr"          : 0x12,
    "set_steering"     : 0x14,
    "set_brake"        : 0x16,
    "set_speed"        : 0x1A,
    "set_leds"         : 0x1C,
    "get_power"        : 0x1E,
    "kill"             : 0x20,
}
