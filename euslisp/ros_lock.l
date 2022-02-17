(ros::load-ros-manifest "ros_lock")

(defclass ros-lock
  :super propertied-object
  :slots (lock-name
          srvname-acquire
          srvname-release))

(defmethod ros-lock
  (:init (lockname
          &key
           (clientname (ros::get-name))
           (service-name-acquire "/lock_manager/acquire")
           (service-name-release "/lock_manager/release"))
    (setq lock-name lockname)
    (setq client-name clientname)
    (setq srvname-acquire service-name-acquire)
    (setq srvname-release service-name-release)
    )

  (:wait-for-server (&optional (timeout nil))
    (let (ret1 ret2)
      (if timeout
        (progn
          (setq ret1 (ros::wait-for-service srvname-acquire timeout))
          (setq ret2 (ros::wait-for-service srvname-release timeout)))
        (progn
          (setq ret1 (ros::wait-for-service srvname-acquire))
          (setq ret2 (ros::wait-for-service srvname-release))))
      (and ret1 ret2)))

  (:acquire (&key
                (timeout -1))
    (let (req res)
      (setq req (instance ros_lock::AcquireRequest :init))
      (send req :lock_name lock-name)
      (send req :client_name client-name)
      (send req :timeout timeout)
      (setq res (ros::service-call srvname-acquire req t))
      (send res :success)))

  (:release ()
    (let (req res)
      (setq req (instance ros_lock::ReleaseRequest :init))
      (send req :lock_name lock-name)
      (send req :client_name client-name)
      (setq res (ros::service-call srvname-release req t))
      (send res :success)))
  )
