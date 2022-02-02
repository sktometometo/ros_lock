#include "ros/ros.h"
#include "ros_lock/Acquire.h"
#include "ros_lock/Release.h"

namespace ros_lock {

class ROSLock {

public:

    ROSLock();
    ~ROSLock();

    void lock();
    bool try_lock();
    void unlock();

private:

    ros::ServiceClient<ros_lock::Acquire> client_acquire;
    ros::ServiceClient<ros_lock::Release> client_release;
    std::string lock_name;
    std::string client_name;

};

class lock_guard {
};

} // end of ros_lock
