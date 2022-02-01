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

  (:acquire (&key
                (timeout -1))
    (let (req res)
      (setq req (instance ros_lock::Acquire :init))
      (send req :lock-name lock-name)
      (send req :client-name client-name)
      (send req :timeout timeout)
      (setq res (ros::service-call service-name-acquire req t))
      (send res :success)))

  (:release ()
    (let (req res)
      (setq req (instance ros_lock::Release :init))
      (send req :lock-name lock-name)
      (send req :client-name client-name)
      (setq res (ros::service-call service-name-release req t))
      (send res :success)))
  )
