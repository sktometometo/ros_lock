#!/usr/bin/env python

import rospy
from ros_lock.lock_manager import LockManager


if __name__ == '__main__':

    rospy.init_node('lock_manager')
    node = LockManager()
    node.spin()
