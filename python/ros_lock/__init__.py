import rospy

from ros_lock.srv import Acquire
from ros_lock.srv import AcquireRequest
from ros_lock.srv import Release
from ros_lock.srv import ReleaseRequest
from contextlib import contextmanager


@contextmanager
def roslock_acquire(roslock, timeout=-1):
    result = roslock.acquire(timeout=timeout)
    yield result
    if result:
        roslock.release()


class ROSLock(object):

    def __init__(self,
                 lock_name='sample',
                 client_name=None,
                 service_name_acquire='/lock_manager/acquire',
                 service_name_release='/lock_manager/release'):

        self.lock_name = lock_name
        if client_name is None:
            self.client_name = rospy.get_name()
        else:
            self.client_name = client_name

        self.service_name_acquire = service_name_acquire
        self.service_name_release = service_name_release

        self.srvclient_acquire = rospy.ServiceProxy(
            service_name_acquire,
            Acquire
        )
        self.srvclient_release = rospy.ServiceProxy(
            service_name_release,
            Release
        )

    def wait_for_server(self, timeout=None):

        rospy.wait_for_service(self.service_name_acquire, timeout=timeout)
        rospy.wait_for_service(self.service_name_release, timeout=timeout)

    def acquire(self, timeout=-1):

        req = AcquireRequest()
        req.lock_name = self.lock_name
        req.client_name = self.client_name
        req.timeout = timeout
        res = self.srvclient_acquire(req)
        return res.success

    def release(self):

        req = ReleaseRequest()
        req.lock_name = self.lock_name
        req.client_name = self.client_name
        res = self.srvclient_release(req)
        return res.success

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self.release()
