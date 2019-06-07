#!/usr/bin/env python

import tests
import rospy
import unittest
PKG = 'rosinterface_handler'

if __name__ == '__main__':
    import rostest

    rospy.init_node(PKG)
    rostest.rosrun(PKG, "RosinterfaceTestSuite", "tests.RosinterfaceTestSuite")
