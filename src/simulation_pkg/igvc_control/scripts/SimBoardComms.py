#!/usr/bin/python
"""
comms_sub

Listens to board communication messages and adapts them to be sent to gazebo 
"""

import rospy
from std_msgs.msg import UInt16, Float64

def _map(inval, inmin, inmax, outmin, outmax):
    return (((float(inval) - inmin) / (inmax - inmin)) * (outmax - outmin)) + outmin

def _clip(inval, inmin, inmax):
    return min(max(inval, inmin), inmax)

class SimBoardComms:

    _MAX_STEER = 0.785
    _MAX_VELOCITY = 80

    def __init__(self):

        self._left_speed_pub = rospy.Publisher('/igvc/left_rear_velocity_controller/command/data', Float64, queue_size=10)
        self._right_speed_pub = rospy.Publisher('/igvc/right_rear_velocity_controller/command/data', Float64, queue_size=10)
        self._left_steer_pub = rospy.Publisher('/igvc/left_steering_position_controller/command/data', Float64, queue_size=10)
        self._right_steer_pub = rospy.Publisher('/igvc/right_steering_position_controller/command/data', Float64, queue_size=10)

        self._current_set_speed = 0
        self._current_set_steering = 0
        self._current_set_brake = 0

        rospy.init_node("SimBordComms", anonymous=True)

        rospy.Subscriber('Set_Speed', UInt16, self._set_speed_callback)
        rospy.Subscriber('Set_Steering', UInt16, self._set_steering_callback)
        rospy.Subscriber('Set_Brake', UInt16, self._set_brake_callback)

        rospy.loginfo("Started SimBoardComms translation node")
    
    def _translate_steering(self, steering):
        return _map(steering, 45, 135, -1, 1)

    def update(self):
        self.rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            steering = self._translate_steering(self._current_set_steering)
            velocity = 0 if self._current_set_brake is not 0 else self._current_set_speed

            self.set_steering(steering)
            self.set_velocity(velocity)

            try:
                self.rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                rospy.loginfo("SimBoardComms shutting down...")
    
    def _set_speed_callback(self, data):
        self._current_set_speed = data.data
    
    def _set_steering_callback(self, data):
        self._current_set_steering = data.data

    def _set_brake_callback(self, data):
        self._current_set_brake = data.data
    
    def set_steering(self, percent):
        """
        Set the wheel positions
        
        percent -- The angle to set the wheels to. Ranges from -1 to 1, with 0 being straight forward
        """

        angle = _map(_clip(percent, -1, 1), -1, 1, -self._MAX_STEER, self._MAX_STEER)
        self._left_steer_pub.publish(angle)
        self._right_steer_pub.publish(angle)
    
    def set_velocity(self, velocity):
        """
        Set the velocity

        percent -- The velocity the cart should move at
        """

        velocity = _clip(velocity, 0, self._MAX_VELOCITY)
        self._left_speed_pub.publish(velocity)
        self._right_speed_pub.publish(velocity)

if __name__ == "__main__":
    comms = SimBoardComms()
    comms.update()