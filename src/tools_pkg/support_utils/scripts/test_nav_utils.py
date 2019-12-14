#!/usr/bin/env python
import rostest
import unittest
import numpy as np
import support_nav_utils.nav_utils as nav_utils
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Vector3
from igvc_msgs.msg import Path, PathPoint, Trajectory, TrajectoryPoint


class NavUtilsTests(unittest.TestCase):
    def setUp(self):
        self.path_point = PathPoint(Header(), Point(24,10,0), Vector3(0,0,0))
        self.traj_point = TrajectoryPoint(Header(), Point(24,10,0), Vector3(0,0,0), Vector3(0,0,0)) 
        self.list = [24,10,0]
        self.array = np.array([24,10,0])

    def test_convert_path_point(self):
        self.assertTrue(np.array_equal(nav_utils.convert_point_type(self.path_point), self.array))

    def test_convert_trajectory_point(self):
        self.assertTrue(np.array_equal(nav_utils.convert_point_type(self.traj_point), self.array))

    def test_convert_list_point(self):
        self.assertTrue(np.array_equal(nav_utils.convert_point_type(self.list), self.array))

    def test_convert_array_point(self):
        self.assertTrue(np.array_equal(nav_utils.convert_point_type(self.array), self.array)) 


if __name__ == '__main__':
    rostest.rosrun('support_utils', 'test_nav_utils', NavUtilsTests)

