#!/usr/bin/env python

import rospy

from ros_lock import ROSLock


if __name__ == '__main__':

    rospy.init_node('sample_client_rospy')

    num_iteration = rospy.get_param('~num_iteration', 10)
    use_ros_lock = rospy.get_param('~use_ros_lock', True)

    rospy.sleep(5)

    ros_lock = ROSLock('sample')

    def lock_hook():
        ros_lock.release()

    rate = rospy.Rate(10)
    if use_ros_lock:
        for i in range(num_iteration):
            rate.sleep()
            with ros_lock:
                rospy.loginfo('fuga {}'.format(i))
                rate.sleep()
                rospy.loginfo('fuga {}'.format(i))
    else:
        for i in range(num_iteration):
            rate.sleep()
            rospy.loginfo('fuga {}'.format(i))
            rate.sleep()
            rospy.loginfo('fuga {}'.format(i))
