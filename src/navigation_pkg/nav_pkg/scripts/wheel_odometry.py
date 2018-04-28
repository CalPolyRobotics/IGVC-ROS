#!/usr/bin/env python
from math import sin, cos
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import UInt16MultiArray


def vel_callback(data):
    """Callback for velocity update"""
    global g_last_vel_time, g_vel_x, g_vel_y, g_vel_dt
    current_time = rospy.Time.now()

    v_left = (data.data[0] - 25) / 1000.0     # Left wheel data
    v_right = (data.data[1] - 25) / 1000.0    # Right wheel data

    g_vel_x = (v_left + v_right) / 2          # Average Velocity
    print("Avg Velocity is %d", g_vel_x)
    g_vel_y = 0  # No y-axis velocity for the golf cart

    g_vel_dt = (current_time - g_last_vel_time).to_sec()
    g_last_vel_time = current_time
    update_vel()


def update_vel():
    "Main loop"""
    global g_last_loop_time, g_vel_x, g_vel_y, g_vel_dt, x_pos, y_pos
    #x_pos = 0.0
    #y_pos = 0.0

    #while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    linear_velocity_x = g_vel_x
    linear_velocity_y = g_vel_y

    # Calculate current position of the robot
    x_pos += linear_velocity_x * g_vel_dt
    y_pos += linear_velocity_y * g_vel_dt

    # All odometry is 6DOF, so need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x_pos, y_pos, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_footprint"
    odom.twist.twist = Twist(
        Vector3(linear_velocity_x, linear_velocity_y, 0), Vector3(0, 0, 0))

    # publish the message
    odom_pub.publish(odom)

    g_last_loop_time = current_time
    #r.sleep()


if __name__ == "__main__":

    rospy.init_node('odometry_publisher')

    #current_time = rospy.Time.now()
    g_last_loop_time = rospy.Time.now()
    g_last_vel_time = rospy.Time.now()  # 0?

    g_vel_x = 0.0
    g_vel_y = 0.0
    g_vel_dt = 0.0

    x_pos = 0.0
    y_pos = 0.0

    #r = rospy.Rate(15.0)

    odom_pub = rospy.Publisher("wheel_odom", Odometry, queue_size=50)
    vel_sub = rospy.Subscriber(
        "Get_Speed", UInt16MultiArray, vel_callback)

    #main()
    rospy.spin()
