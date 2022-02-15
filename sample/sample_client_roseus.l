#!/usr/bin/env roseus

(load "package://ros_lock/euslisp/ros_lock.l")


(ros::roseus "sample_client_roseus")

(setq num-iteration (ros::get-param "~num_iteration" 10))
(setq use-ros-lock (ros::get-param "~use_ros_lock" t))
(setq duration-sleep (ros::get-param "~duration_sleep" 0.1))

(unix:sleep 5)

(setq *ros-lock* (instance ros-lock :init "sample"))

(if use-ros-lock
    (dotimes (i num-iteration)
      (unix:usleep (round (* 1000000 duration-sleep)))
      (send *ros-lock* :acquire)
      (ros::ros-info "start ~A th iteration" i)
      (unix:usleep (round (* 1000000 duration-sleep)))
      (ros::ros-info "stop ~A th iteration" i)
      (send *ros-lock* :release))
  (dotimes (i num-iteration)
    (unix:usleep (round (* 1000000 duration-sleep)))
    (ros::ros-info "start ~A th iteration" i)
    (unix:usleep (round (* 1000000 duration-sleep)))
    (ros::ros-info "stop ~A th iteration" i))
  )

(exit)
