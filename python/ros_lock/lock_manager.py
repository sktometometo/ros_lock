import threading

import rospy
from ros_lock.srv import Acquire
from ros_lock.srv import AcquireResponse
from ros_lock.srv import Release
from ros_lock.srv import ReleaseResponse


class LockManager(object):

    def __init__(self):

        self.lock_for_lock_list = threading.Lock()
        self.lock_list = {}
        self.lock_for_client_names = threading.Lock()
        self.client_names = {}

        self.srv_acquire = rospy.Service('~acquire', Acquire, self.handler_acquire)
        self.srv_release = rospy.Service('~release', Release, self.handler_release)

    def handler_acquire(self, srv):

        with self.lock_for_lock_list:
            if srv.lock_name not in self.lock_list:
                self.lock_list[srv.lock_name] = threading.Lock()

        # if specified lock is acquired
        # wait until lock is released
        ret = self.lock_list[srv.lock_name].acquire(srv.timeout)
        with self.lock_for_client_names:
            if ret:
                self.client_names[srv.lock_name] = srv.client_name
        res = AcquireResponse()
        res.success = ret

        return res

    def handler_release(self, srv):

        with self.lock_for_lock_list, self.lock_for_client_names:
            if srv.lock_name not in self.lock_list \
                    and srv.lock_name not in self.client_names \
                    and self.client_names[srv.lock_name] != srv.client_name:
                res = ReleaseResponse()
                res.success = False
                return res

        with self.lock_for_client_names:
            self.client_names[srv.lock_name] = None
            ret = self.lock_list[srv.lock_name].release()

        res = ReleaseResponse()
        res.success = True
        return res
