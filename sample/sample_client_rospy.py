#!/usr/bin/env python

import rospy

from ros_lock import ROSLock


if __name__ == '__main__':

    rospy.init_node('sample_client_rospy')

    num_iteration = rospy.get_param('~num_iteration', 10)
    use_ros_lock = rospy.get_param('~use_ros_lock', True)
    duration_sleep = rospy.get_param('~duration_sleep', 0.1)

    rospy.sleep(5)

    ros_lock = ROSLock('sample')

    def lock_hook():
        ros_lock.release()

    if use_ros_lock:
        for i in range(num_iteration):
            rospy.sleep(duration_sleep)
            with ros_lock:
                rospy.loginfo('start {} th iteration'.format(i))
                rospy.sleep(duration_sleep)
                rospy.loginfo('stop {} th iteration'.format(i))
    else:
        for i in range(num_iteration):
            rospy.sleep(duration_sleep)
            rospy.loginfo('start {} th iteration'.format(i))
            rospy.sleep(duration_sleep)
            rospy.loginfo('stop {} th iteration'.format(i))
