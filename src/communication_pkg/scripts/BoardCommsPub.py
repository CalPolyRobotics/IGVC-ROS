"""
BoardCommsPub

Publisher functions for publishing data from communication with the golf cart
"""

import rospy
from std_msgs.msg import UInt8, UInt8MultiArray, UInt16MultiArray, UInt16

STATUS = rospy.Publisher("Status_Response", UInt8MultiArray, queue_size=1000)
SPEED = rospy.Publisher("Get_Speed", UInt16MultiArray, queue_size=1000)
PEDAL = rospy.Publisher("Get_Pedal", UInt16, queue_size=1000)
SONAR = rospy.Publisher("Get_Sonar", UInt16, queue_size=1000)
FNR = rospy.Publisher("Get_FNR", UInt8, queue_size=1000)
AMAN = rospy.Publisher("Get_AMan", UInt8, queue_size=1000)
STEERING = rospy.Publisher("Get_Steering", UInt16, queue_size=1000)
POWER = rospy.Publisher("Get_Power", UInt16MultiArray, queue_size=1000)

def status_resp(data):
    """
    status_resp

    Publishes the data from a status response
    """
    status = UInt8MultiArray()
    status.data = list(data)

    STATUS.publish(status)

def get_speed_resp(data):
    """
    get_speed_resp

    Publishes the data from a get_speed_resp
    """
    speed = UInt16MultiArray()
    for i in range(0, len(data), 2):
        speed.data.append((data[i] << 8) | data[i+1])
    SPEED.publish(speed)

def get_pedal_resp(data):
    """
    get_pedal_resp

    Publishes the data from a get_pedal_resp
    """
    pedal = (data[0] << 8) | data[1]
    PEDAL.publish(pedal)

def get_sonar_resp(data):
    """
    get_sonar_resp

    Publishes the data from a get_sonar_resp
    """

    sonar = UInt16MultiArray()
    for i in range(0, len(data), 2):
        sonar.data.append((data[i] << 8) | data[i+1])
    SONAR.publish(sonar)

def get_fnr_resp(data):
    """
    get_fnr_resp

    Publishes the data from a get_fnr_resp
    """
    FNR.publish(data[0])

def get_aman_resp(data):
    """
    get_aman_resp

    Publishes the data from a get_aman_resp
    """
    AMAN.publish(data[0])

def get_steering_resp(data):
    """
    get_steering_resp

    Publishes the data from a get_steering_resp
    """
    steer = (data[1] <<8 | data[0])
    STEERING.publish(steer)

def get_power_resp(data):
    """
    get_power_resp

    Publishes the data from a get_power_resp
    """
    power = UInt16MultiArray()
    for i in range(0, len(data), 2):
        power.data.append((data[i] << 8) | data[i+1])
    POWER.publish(power)

PUB_CALLBACK_LUT = {
    0x01 : status_resp,
    0x03 : get_speed_resp,
    0x05 : get_pedal_resp,
    0x07 : get_sonar_resp,
    0x0F : get_fnr_resp,
    0x11 : get_aman_resp,
    0x19 : get_steering_resp,
    0x1F : get_power_resp
}
