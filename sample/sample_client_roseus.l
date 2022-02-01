#!/usr/bin/env roseus

(load "package://roseus_lock/euslisp/roseus_lock.l")


(ros::roseus "sample_client_roseus")

(setq *ros-lock* (instance ros-lock :init "sample"))

(dotimes (i 10)
  (send *ros-lock* :acquire)
  (ros::ros-info "hoge")
  (ros::ros-info "hoge")
  (send *ros-lock* :release)
  )
