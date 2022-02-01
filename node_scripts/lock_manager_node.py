#!/usr/bin/env python

import rospy
from roseus_lock.lock_manager import LockManager


if __name__ == '__main__':

    rospy.init_node('lock_manager')
    node = LockManager()
    rospy.spin()
