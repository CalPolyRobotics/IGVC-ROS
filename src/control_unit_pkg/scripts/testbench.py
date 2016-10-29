import rospy
import time
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import UInt8
from std_msgs.msg import UInt16


PUB_FNR = rospy.Publisher('Set_FNR', UInt8, queue_size=10)
PUB_THR = rospy.Publisher('Set_Throttle', UInt16, queue_size=10)
PUB_SPEED = rospy.Publisher("Set_Speed", UInt16MultiArray, callbackSpeed)
#PUB_FNR = rospy.Publisher("Get_FNR", UInt8, callbackFNR)

rospy.Publisher("lidar_scan_ranges", Float32MultiArray, callbackLidar)
