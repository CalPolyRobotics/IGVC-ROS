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

DATA_TYPES = (
   "GET_STATUS", 
   "R_GET_STATUS", 
   "GET_SONAR_1", 
   "R_GET_SONAR_1", 
   "GET_SONAR", 
   "R_GET_SONAR",
   "SET_FNR",  
   "R_SET_FNR",
   "GET_FNR",
   "R_GET_FNR",
	"SET_THROTTLE", 
   "R_SET_THROTTLE",
   "SET_SPEED", 
   "R_SET_SPEED",
   "GET_SPEED", 
   "R_GET_SPEED",
   "SET_STEERING", 
   "R_SET_STEERING",
	"GET_STEERING", 
   "R_GET_STEERING",
   "SET_LIGHTS", 
   "R_SET_LIGHTS",
   "GET_BATTERY", 
   "R_GET_BATTERY",
   "GET_POWER",
   "R_GET_POWER",
   "STOP",  
   "R_STOP")

DATA_CODES ={ 
   "GET_STATUS"     : (0x00, 0), 
   "GET_SONAR_1"    : (0x02, 0), 
   "GET_SONAR"      : (0x04, 0), 
   "SET_FNR"        : (0x06, 1), 
	"GET_FNR"        : (0x08, 0), 
   "SET_THROTTLE"   : (0x0A, 2),
   "SET_SPEED"      : (0x0C, 2), 
   "GET_SPEED"      : (0x0E, 0), 
   "SET_STEERING"   : (0x10, 2), 
	"GET_STEERING"   : (0x12, 0), 
   "SET_LIGHTS"     : (0x14, 2), 
   "GET_BATTERY"    : (0x16, 0),
   "GET_POWER"      : (0x18, 0),
   "STOP"           : (0x1A, 0), 

	"R_GET_STATUS"   : (0x01, 0), 
   "R_GET_SONAR_1"  : (0x03, 2), 
   "R_GET_SONAR"    : (0x05, 12), 
   "R_SET_FNR"      : (0x07, 1),
   "R_GET_FNR"      : (0x09, 1),
	"R_SET_THROTTLE" : (0x0B, 1), 
   "R_SET_SPEED"    : (0x0D, 1), 
   "R_GET_SPEED"    : (0x0F, 4), 
   "R_SET_STEERING" : (0x11, 2),
	"R_GET_STEERING" : (0x13, 2), 
   "R_SET_LIGHTS"   : (0x15, 2), 
   "R_GET_BATTERY"  : (0x17, 4), 
   "R_GET_POWER"    : (0x19, 16),
   "R_STOP"         : (0x1B, 1)}

DATA_LENGTH ={
   0x01: DATA_CODES["R_GET_STATUS"   ][1],
   0x03: DATA_CODES["R_GET_SONAR_1"  ][1],
   0x05: DATA_CODES["R_GET_SONAR"    ][1],
   0x07: DATA_CODES["R_SET_FNR"      ][1],
   0x09: DATA_CODES["R_GET_FNR"      ][1],
   0x0B: DATA_CODES["R_SET_THROTTLE" ][1],
   0x0D: DATA_CODES["R_SET_SPEED"    ][1],
   0x0F: DATA_CODES["R_GET_SPEED"    ][1],
   0x11: DATA_CODES["R_SET_STEERING" ][1],
   0x13: DATA_CODES["R_GET_STEERING" ][1],
   0x15: DATA_CODES["R_SET_LIGHTS"   ][1],
   0x17: DATA_CODES["R_GET_BATTERY"  ][1],
   0x19: DATA_CODES["R_GET_POWER"    ][1],
   0x1B: DATA_CODES["R_STOP"         ][1]}
