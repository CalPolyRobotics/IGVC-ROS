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
        self.path_point1 = PathPoint(Header(), Point(24.,10.,0.), Vector3(0.,0.,0.))
        self.path_point2 = PathPoint(Header(), Point(5.,5.,0.), Vector3(0.,0.,0.))
        self.traj_point1 = TrajectoryPoint(Header(), Point(24.,10.,0.), Vector3(0.,0.,0.), Vector3(0.,0.,0.))
        self.traj_point2 = TrajectoryPoint(Header(), Point(5.,5.,0.), Vector3(0.,0.,0.), Vector3(0.,0.,0.))
        self.point_list = [24.,10.,0.]
        self.point_array = np.array([24.,10.,0.])
        self.path = Path(Header(), [self.path_point1, self.path_point2, self.path_point1])
        self.path_array = np.array([[[24.,10.,0.], [0.,0.,0.]], [[5.,5.,0.], [0.,0.,0.]], [[24.,10.,0.], [0.,0.,0.]]])
        self.trajectory = Trajectory(Header(), [self.traj_point1, self.traj_point2, self.traj_point1])
        self.traj_array = np.array([[[24.,10.,0.], [0.,0.,0.], [0.,0.,0.]], [[5.,5.,0.], [0.,0.,0.], [0.,0.,0.]], [[24.,10.,0.], [0.,0.,0.], [0.,0.,0.]]])
        self.array = np.array([[24.,10.,0.], [5.,5.,0.], [24.,10.,0.]])
        self.list = [[24.,10.,0.], [5.,5.,0.], [24.,10.,0.]]
        self.test_path = [[1.,1.,0], [2.,4.,0.], [3.,9.,0.], [4.,16.,0.], [5.,25.,0]]
        self.test_point1 = [2.,3.5,0.]
        self.test_point2 = [2.2,5.5,0.]
        self.test_point3 = [4.5,24.,0.]

    def test_convert_path_point(self):
        """Test for converting a PathPoint msg to a numpy array"""
        self.assertTrue(np.array_equal(nav_utils.convert_point_type(self.path_point1), self.point_array))

    def test_convert_trajectory_point(self):
        """Test for converting a TrajectoryPoint msg to a numpy array"""
        self.assertTrue(np.array_equal(nav_utils.convert_point_type(self.traj_point1), self.point_array))

    def test_convert_list_point(self):
        """Test for converting a list to a numpy array"""
        self.assertTrue(np.array_equal(nav_utils.convert_point_type(self.point_list), self.point_array))

    def test_convert_array_point(self):
        """Test for converting a numpy array to a numpy array"""
        self.assertTrue(np.array_equal(nav_utils.convert_point_type(self.point_array), self.point_array))

    def test_path_to_array(self):
        """Test for converting a Path msg to a numpy array"""
        self.assertTrue(np.array_equal(nav_utils.path_to_array(self.path), self.path_array))

    def test_trajectory_to_array(self):
        """Test for converting a Trajectory msg to a numpy array"""
        self.assertTrue(np.array_equal(nav_utils.trajectory_to_array(self.trajectory), self.traj_array))

    def test_array_to_path(self):
        """Test for converting a numpy array to a Path msg"""
        self.assertEqual(nav_utils.array_to_path(self.path_array), self.path)

    def test_array_to_trajectory(self):
        """Test for converting a numpy array to a Trajectory msg"""
        self.assertEqual(nav_utils.array_to_trajectory(self.traj_array), self.trajectory)

    def test_convert_path_type_with_path(self):
        """Test for converting a Path msg to a numpy array (dropping orientation info)"""
        self.assertTrue(np.array_equal(nav_utils.convert_path_type(self.path), self.array))

    def test_convert_path_type_with_trajectory(self):
        """Test for converting a Trajectory msg to a numpy array (dropping orientation & velocity info)"""
        self.assertTrue(np.array_equal(nav_utils.convert_path_type(self.trajectory), self.array))

    def test_convert_path_type_with_list(self):
        """Test for converting a list to a numpy array"""
        self.assertTrue(np.array_equal(nav_utils.convert_path_type(self.list), self.array))

    def test_convert_path_type_with_array(self):
        """Test for converting a numpy array to a numpy array"""
        self.assertTrue(np.array_equal(nav_utils.convert_path_type(self.array), self.array))

    def test_calculate_distance_same_point_type(self):
        """Test for calculating the distance between two points of same type"""
        self.assertAlmostEqual(nav_utils.calculate_distance(self.path_point1, self.path_point2), 19.6468827)

    def test_calculate_distance_diff_point_type(self):
        """Test for calculating the distance between two points of different type"""
        self.assertAlmostEqual(nav_utils.calculate_distance(self.path_point1, self.traj_point2), 19.6468827)

    def test_calculate_yaw_error_in_bounds(self):
        """Test for caculating the yaw error between path orienation and vehicle state when error is in [-pi,pi]"""
        self.assertAlmostEqual(nav_utils.calculate_yaw_error(np.pi, np.pi/4), 3*np.pi/4)

    def test_calculate_yaw_error_out_bounds_neg(self):
        """Test for caculating the yaw error between path orienation and vehicle state when error is <-pi"""
        self.assertAlmostEqual(nav_utils.calculate_yaw_error(-3*np.pi/4, np.pi/2), 3*np.pi/4)

    def test_calculate_yaw_error_out_bounds_pos(self):
        """Test for caculating the yaw error between path orienation and vehicle state when error is >pi"""
        self.assertAlmostEqual(nav_utils.calculate_yaw_error(np.pi/2, -3*np.pi/4), -3*np.pi/4)

    def test_get_closest_point_in_front(self):
        """Test for getting the next closest point when the closest point is in front of current point"""
        self.assertEqual(nav_utils.get_closest_point(self.test_path, self.test_point1), ([2.,4.,0.], 1))

    def test_get_closest_point_behind(self):
        """Test for getting the next closest point when the closest point is behind the current point"""
        self.assertEqual(nav_utils.get_closest_point(self.test_path, self.test_point2), ([3.,9.,0.], 2))

    def test_get_closest_point_last(self):
        """Test for getting the next closest point when the closest point is the last point"""
        self.assertEqual(nav_utils.get_closest_point(self.test_path, self.test_point3), ([5.,25.,0.], 4))


if __name__ == '__main__':
    rostest.rosrun('support_utils', 'test_nav_utils', NavUtilsTests)
