#!/usr/bin/env roseus

(load "package://ros_lock/euslisp/ros_lock.l")


(ros::roseus "sample_client_roseus")

(setq num-iteration (ros::get-param "~num_iteration" 10))
(setq use-ros-lock (ros::get-param "~use_ros_lock" t))

(unix:sleep 5)

(setq *ros-lock* (instance ros-lock :init "sample"))

(ros::rate 10)
(if use-ros-lock
    (dotimes (i num-iteration)
      (ros::sleep)
      (send *ros-lock* :acquire)
      (ros::ros-info "hoge ~A" i)
      (ros::sleep)
      (ros::ros-info "hoge ~A" i)
      (send *ros-lock* :release))
  (dotimes (i num-iteration)
    (ros::sleep)
    (ros::ros-info "hoge ~A" i)
    (ros::sleep)
    (ros::ros-info "hoge ~A" i))
  )

(exit)
