import rospy
import tf
from sensor_msgs.msg import NavSatFix , Imu 
from nav_msgs.msg import Odometry
#from igvc_msgs.msg import Path , PathPoint
#from std_msgs import Bool
#import utm
import math
#import nav_utils

MAX_ANGLE = math.pi/6

class StanleySteering:
    '''
    Post a steering angle given the current position
    the current target waypoint and the next waypoint
    '''
    def __init__(self):
        k = 1
        velocity = 0
        path = []
        target_point = (0,0,0) #(x,y,z)
        target_orientation = (0,0,0) #(roll,pitch,yaw)
        current_pos = (0,0)

        angle_error = 0
        crosstracking_error = 0

        speed = 5
        steering_angle = 0

        rospy.init_node('stanley_steering', anonymous = True)
        steering_pub = rospy.Publisher('Set_Steering', UInt16, queue_size = 1)
        speed_pub = rospy.Publisher('Set_Speed', UInt16, queue_size = 1)
        completion_pub = rospy.Publisher('at_goal', Bool,  queue_size = 1)
        rospy.Subscriber('odom' , Odometry, self.update_velocity)
        rospy.Subscriber('imu/data', Imu, update_angle)
        rospy.Subscriber('fix', NavSatFix, self.update_position)
        rospy.Subscriber('path', Path,  self.update_path)

    def update_angle(self, data):   
        '''
        call back for Imu data

        Args:
            data: an odometry message that also updates the 
        '''

        orientation = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        euler_angles = tf.transformations.euler_from_quiaternion(orientation)
        self.angle_error = self.target_orientation[2] - euler_angles[2]


    def update_velocity(self, data):
        '''
        call back from odometry velocity data

        Args: 
            data: an odometry message
        '''
        self.velocity = data.linear.x


    def update_position(self, data):
        '''
        call back from gps data for position that also updates the cross tracking error

        Args:
            data: a NavSatFix message
        '''

        x,y,_,_ = utm.from_lat_lon(data.latitude, data.longitude)
        self.current_pos = (x,y)
        self.crosstracking_error = self.distance(current_pos, target_point) ** 2


    def update_path(self, path):
        '''
        call back from paths

        Args:
            path: path message
        '''
        self.path = path



    def set_init_point(self):
        self.target_point = nav_utils.get_closest_point(self.path, current_pos)[0].point
        self.target_orientation = nav_utils.get_closest_point(self.path, current_pos)[0].orientation
      
    def distance(P1 , P2):
        return ((P2[0] - P1[0])**2 + (P2[1] - P1[1])**2) ** .5

    def update_target_pos(self):
        index = nav_utils.get_closest_point(self.path, current_pos)[1]
        next_point = self.path[index + 1].point

        if(self.distance(self.current_pos , next_point) < .5):
            self.target_point = next_point.point
            self.target_orientation = next_point.orientation
            

    def set_steering_angle(self):
        '''
        updates the steering angle based on the current steering angle
        '''

        self.steering_angle = self.angle_error + math.atan(self.k * self.crosstracking_error / self.velocity)

    def adjust_steering(self):
        self.set_init_point()
        while(nav_utils.get_closest_point(self.path , current_pos)[1] < len(path) - 1):
            self.update_target_pos()
            self.set_steering_angle()
            angle = math.degrees(self.steering_angle)
            angle = min(angle, MIN_ANGLE) if angle > 0 else max(angle, -1* MIN_ANGLE)
            self.steering_pub = angle
            self.speed_pub = self.speed

if __name__ == "__main__":
    stanley_steering = StanleySteering()
    stanley_steering.adjust_steering()


