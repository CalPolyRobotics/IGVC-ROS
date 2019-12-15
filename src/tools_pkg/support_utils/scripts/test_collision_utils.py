#!/usr/bin/env python
import rostest
import unittest
import numpy as np
import support_nav_utils.collision_utils as collision_utils


class CollisionUtilsTests(unittest.TestCase):
    def setUp(self):
        self.line_segment1 = [[-4.,4.,0.], [4.,-4.,0.]]  # [start, end]
        self.line_segment2 = [[2.,1.,0.], [-3.,-2.,0.]]
        self.line_segment3 = [[0.,5.,0.], [10.,5.,0.]]
        self.line_segment4 = [[0.,1.,0.], [10.,1.,0.]]
        self.line_segment5 = [[7.,5.,0.], [12.,5.,0.]]

    def test_check_intersection_true(self):
        """Test if two line segments intersect each other (case when they do)"""
        self.assertTrue(collision_utils.check_intersection(self.line_segment1[0], self.line_segment1[1],
                                                           self.line_segment2[0], self.line_segment2[1]))
        self.assertTrue(collision_utils.check_intersection(self.line_segment4[0], self.line_segment4[1],
                                                           self.line_segment2[0], self.line_segment2[1]))
        self.assertTrue(collision_utils.check_intersection(self.line_segment3[0], self.line_segment3[1],
                                                           self.line_segment5[0], self.line_segment5[1]))

    def test_check_intersection_false(self):
        """Test if two line segments intersect each other (case when they don't)"""
        self.assertFalse(collision_utils.check_intersection(self.line_segment1[0], self.line_segment1[1],
                                                            self.line_segment3[0], self.line_segment3[1]))

    def test_calculate_orientation_collinear(self):
        """Test calculating the orientation of an ordered triplet (case when orientation is collinear)"""
        self.assertEqual(collision_utils.calculate_orientation(self.line_segment3[0], self.line_segment3[1], self.line_segment5[0]), 0)

    def test_calculate_orientation_clockwise(self):
        """Test calculating the orientation of an ordered triplet (case when orientation is clockwise)"""
        self.assertEqual(collision_utils.calculate_orientation(self.line_segment3[0], self.line_segment3[1], self.line_segment1[1]), 1)

    def test_calculate_orientation_counter_clockwise(self):
        """Test calculating the orientation of an ordered triplet (case when orientation is counter clockwise)"""
        self.assertEqual(collision_utils.calculate_orientation(self.line_segment4[0], self.line_segment4[1], self.line_segment1[0]), 2)

    def test_check_on_segment_true(self):
        """Test if two parrallel line segments are collinear (case when they are)"""
        self.assertTrue(collision_utils.check_on_segment(self.line_segment3[0], self.line_segment3[1], self.line_segment5[1]))

    def test_check_on_segment_false(self):
        """Test if two parrallel line segments are collinear (case when they are not)"""
        self.assertFalse(collision_utils.check_on_segment(self.line_segment3[0], self.line_segment3[1], self.line_segment4[0]))

if __name__ == '__main__':
    rostest.rosrun('support_utils', 'test_collision_utils', CollisionUtilsTests)
