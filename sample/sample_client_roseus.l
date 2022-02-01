#!/usr/bin/env roseus

(load "package://ros_lock/euslisp/ros_lock.l")


(ros::roseus "sample_client_roseus")

(setq *ros-lock* (instance ros-lock :init "sample"))

(dotimes (i 10)
  (send *ros-lock* :acquire)
  (ros::ros-info "hoge")
  (ros::ros-info "hoge")
  (send *ros-lock* :release)
  )
