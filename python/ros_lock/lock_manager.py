import threading

import rospy
import rosnode
from ros_lock.srv import Acquire
from ros_lock.srv import AcquireResponse
from ros_lock.srv import Release
from ros_lock.srv import ReleaseResponse


class LockEntry(object):

    def __init__(self, lock_name, client_name=None):

        self.lock = threading.Lock()
        self.lock_name = lock_name
        self.client_name = client_name

    def acquire(self, timeout):

        return self.lock.acquire(timeout)

    def release(self):

        return self.lock.release()

    def locked(self):

        return self.lock.locked()

    def is_client_active(self):

        if self.client_name is not None:
            return self.client_name in rosnode.get_node_names()
        else:
            return False


class LockManager(object):

    def __init__(self):

        self.lock_for_lock_list = threading.Lock()
        self.lock_list = {}

        self.srv_acquire = rospy.Service(
            '~acquire', Acquire, self.handler_acquire)
        self.srv_release = rospy.Service(
            '~release', Release, self.handler_release)

    def _acquire(self, lock_name, client_name, timeout, force):

        with self.lock_for_lock_list:
            if lock_name not in self.lock_list:
                self.lock_list[lock_name] = \
                    LockEntry(lock_name)

        # if specified lock is acquired
        # wait until lock is released
        if force and self.lock_list[lock_name].locked():
            self.lock_list[lock_name].release()
        ret = self.lock_list[lock_name].acquire(timeout)
        if ret:
            with self.lock_for_lock_list:
                self.lock_list[lock_name].client_name = client_name

        return ret

    def _release(self, lock_name, client_name):

        with self.lock_for_lock_list:
            if lock_name not in self.lock_list:
                return False, '{} is not in lock_list.'.format(lock_name)

            if self.lock_list[lock_name].client_name != client_name:
                return False, 'Current client name for {} is {}. This is different from release request. ({})'.format(
                    lock_name,
                    self.lock_list[lock_name].client_name,
                    client_name
                )

            if self.lock_list[lock_name].locked():
                self.lock_list[lock_name].release()
                self.lock_list[lock_name].client_name = None
                return True, 'Success'
            else:
                return False, '{} is already released.'.format(lock_name)

    def handler_acquire(self, srv):

        success = self._acquire(srv.lock_name, srv.client_name, srv.timeout, srv.force)
        res = AcquireResponse()
        res.success = success
        return res

    def handler_release(self, srv):

        success, message = self._release(srv.lock_name, srv.client_name)
        res = ReleaseResponse()
        res.success = success
        res.message = message
        return res

    def spin(self, hz=1):

        rate = rospy.Rate(hz)
        while not rospy.is_shutdown():
            rate.sleep()
            with self.lock_for_lock_list:
                for lock_name in self.lock_list:
                    if not self.lock_list[lock_name].is_client_active() \
                            and self.lock_list[lock_name].locked():
                        rospy.logwarn('Client {} for lock {} is not active. released.'.format(
                            self.lock_list[lock_name].client_name,
                            lock_name
                        ))
                        self.lock_list[lock_name].release()
