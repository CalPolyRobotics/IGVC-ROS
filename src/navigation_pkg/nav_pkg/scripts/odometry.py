#!/usr/bin/env python
from math import sin, cos
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Imu

DEBUG = 0


def vel_callback(data):
    """Callback for velocity update"""
    global g_last_vel_time, g_vel_x, g_vel_y, g_vel_dt
    current_time = rospy.Time.now()

    v_left = (data.data[0] - 25) / 1000.0     # Left wheel data
    v_right = (data.data[1] - 25) / 1000.0    # Right wheel data

    g_vel_x = (v_left + v_right) / 2          # Average Velocity
    if DEBUG:
        print("Avg Velocity: %d", g_vel_x)
    g_vel_y = 0  # No y-axis velocity for the golf cart

    g_vel_dt = (current_time - g_last_vel_time).to_sec()
    g_last_vel_time = current_time
    update_vel()


def imu_callback(imu_data):
    """Callback for IMU data update"""
    global g_last_imu_time, g_imu_z, g_imu_dt, IMU_DIRECTION
    current_time = rospy.Time.now()

    if (imu_data.angular_velocity.z > -0.03 and imu_data.angular_velocity.z < 0.03):
        g_imu_z = 0.0
    else:
        g_imu_z = imu_data.angular_velocity.z * IMU_DIRECTION

    if DEBUG:
        print("Angular Velocity: %d", g_imu_z)
    g_imu_dt = (current_time - g_last_imu_time).to_sec()
    g_last_imu_time = current_time


def update_vel():
    "Main loop"""
    global g_last_loop_time, g_vel_x, g_vel_y, g_vel_dt, x_pos, y_pos, theta
    #x_pos = 0.0
    #y_pos = 0.0

    #while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    linear_velocity_x = g_vel_x
    linear_velocity_y = g_vel_y
    angular_velocity = g_imu_z

    delta_theta = angular_velocity * g_imu_dt  # radians
    delta_x = (linear_velocity_x * cos(theta) -
               linear_velocity_y * sin(theta)) * g_vel_dt  # m
    delta_y = (linear_velocity_x * sin(theta) +
               linear_velocity_y * cos(theta)) * g_vel_dt  # m

    # Calculate current position of the robot
    x_pos += delta_x  # linear_velocity_x * g_vel_dt
    y_pos += delta_y  # linear_velocity_y * g_vel_dt
    theta += delta_theta

    # All odometry is 6DOF, so need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

    # First, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x_pos, y_pos, 0.),
        odom_quat,
        current_time,
        "base_footprint",
        "odom"
    )
    # Next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # Set the position
    odom.pose.pose = Pose(Point(x_pos, y_pos, 0.), Quaternion(*odom_quat))

    # Set the velocity
    odom.child_frame_id = "base_footprint"
    odom.twist.twist = Twist(
        Vector3(linear_velocity_x, linear_velocity_y, 0), Vector3(0, 0, g_imu_z))

    # Publish the message
    odom_pub.publish(odom)

    g_last_loop_time = current_time
    #r.sleep()


if __name__ == "__main__":

    rospy.init_node('odometry_publisher')

    #current_time = rospy.Time.now()
    g_last_loop_time = rospy.Time.now()
    g_last_vel_time = rospy.Time.now()  # 0?
    g_last_imu_time = rospy.Time.now()  # 0?
    # Globals for encoder based velocity
    g_vel_x = 0.0
    g_vel_y = 0.0
    g_vel_dt = 0.0

    # Globals for IMU based angular velocity
    g_imu_z = 0.0
    g_imu_dt = 0.0
    IMU_DIRECTION = rospy.get_param(
        'imu_direction', 1)  # +/- 1 to reverse IMU data

    # Globals for position
    x_pos = 0.0
    y_pos = 0.0
    theta = 0.0

    # Publishers, Subscribers, Broadcasters
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    vel_sub = rospy.Subscriber(
        "Get_Speed", UInt16MultiArray, vel_callback)
    imu_sub = rospy.Subscriber("imu/data", Imu, imu_callback)
    odom_broadcaster = tf.TransformBroadcaster()

    # Keep node active
    rospy.spin()
