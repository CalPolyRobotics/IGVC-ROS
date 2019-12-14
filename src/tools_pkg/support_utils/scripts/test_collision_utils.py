#!/usr/bin/env python
import rostest
import unittest
import numpy as np


class CollisionUtilsTests(unittest.TestCase):
    def setUp(self):
        pass

    def test_check_intersection(self):
        pass

if __name__ == '__main__':
    rostest.rosrun('support_utils', 'test_collision_utils', CollisionUtilsTests)
