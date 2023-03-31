;; Auto-generated. Do not edit!


(when (boundp 'tada_ros::MotorDataMsg)
  (if (not (find-package "TADA_ROS"))
    (make-package "TADA_ROS"))
  (shadow 'MotorDataMsg (find-package "TADA_ROS")))
(unless (find-package "TADA_ROS::MOTORDATAMSG")
  (make-package "TADA_ROS::MOTORDATAMSG"))

(in-package "ROS")
;;//! \htmlinclude MotorDataMsg.msg.html


(defclass tada_ros::MotorDataMsg
  :super ros::object
  :slots (_mode _duration _motor1_move _motor2_move _motor1_torque _motor2_torque _PF_cmd _EV_cmd _PF_curr _EV_curr _CPU0 _CPU1 _CPU2 _CPU3 _t ))

(defmethod tada_ros::MotorDataMsg
  (:init
   (&key
    ((:mode __mode) 0)
    ((:duration __duration) 0)
    ((:motor1_move __motor1_move) 0)
    ((:motor2_move __motor2_move) 0)
    ((:motor1_torque __motor1_torque) 0)
    ((:motor2_torque __motor2_torque) 0)
    ((:PF_cmd __PF_cmd) 0.0)
    ((:EV_cmd __EV_cmd) 0.0)
    ((:PF_curr __PF_curr) 0.0)
    ((:EV_curr __EV_curr) 0.0)
    ((:CPU0 __CPU0) 0.0)
    ((:CPU1 __CPU1) 0.0)
    ((:CPU2 __CPU2) 0.0)
    ((:CPU3 __CPU3) 0.0)
    ((:t __t) 0.0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _duration (round __duration))
   (setq _motor1_move (round __motor1_move))
   (setq _motor2_move (round __motor2_move))
   (setq _motor1_torque (round __motor1_torque))
   (setq _motor2_torque (round __motor2_torque))
   (setq _PF_cmd (float __PF_cmd))
   (setq _EV_cmd (float __EV_cmd))
   (setq _PF_curr (float __PF_curr))
   (setq _EV_curr (float __EV_curr))
   (setq _CPU0 (float __CPU0))
   (setq _CPU1 (float __CPU1))
   (setq _CPU2 (float __CPU2))
   (setq _CPU3 (float __CPU3))
   (setq _t (float __t))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:motor1_move
   (&optional __motor1_move)
   (if __motor1_move (setq _motor1_move __motor1_move)) _motor1_move)
  (:motor2_move
   (&optional __motor2_move)
   (if __motor2_move (setq _motor2_move __motor2_move)) _motor2_move)
  (:motor1_torque
   (&optional __motor1_torque)
   (if __motor1_torque (setq _motor1_torque __motor1_torque)) _motor1_torque)
  (:motor2_torque
   (&optional __motor2_torque)
   (if __motor2_torque (setq _motor2_torque __motor2_torque)) _motor2_torque)
  (:PF_cmd
   (&optional __PF_cmd)
   (if __PF_cmd (setq _PF_cmd __PF_cmd)) _PF_cmd)
  (:EV_cmd
   (&optional __EV_cmd)
   (if __EV_cmd (setq _EV_cmd __EV_cmd)) _EV_cmd)
  (:PF_curr
   (&optional __PF_curr)
   (if __PF_curr (setq _PF_curr __PF_curr)) _PF_curr)
  (:EV_curr
   (&optional __EV_curr)
   (if __EV_curr (setq _EV_curr __EV_curr)) _EV_curr)
  (:CPU0
   (&optional __CPU0)
   (if __CPU0 (setq _CPU0 __CPU0)) _CPU0)
  (:CPU1
   (&optional __CPU1)
   (if __CPU1 (setq _CPU1 __CPU1)) _CPU1)
  (:CPU2
   (&optional __CPU2)
   (if __CPU2 (setq _CPU2 __CPU2)) _CPU2)
  (:CPU3
   (&optional __CPU3)
   (if __CPU3 (setq _CPU3 __CPU3)) _CPU3)
  (:t
   (&optional __t)
   (if __t (setq _t __t)) _t)
  (:serialization-length
   ()
   (+
    ;; int32 _mode
    4
    ;; int32 _duration
    4
    ;; int32 _motor1_move
    4
    ;; int32 _motor2_move
    4
    ;; int32 _motor1_torque
    4
    ;; int32 _motor2_torque
    4
    ;; float32 _PF_cmd
    4
    ;; float32 _EV_cmd
    4
    ;; float32 _PF_curr
    4
    ;; float32 _EV_curr
    4
    ;; float32 _CPU0
    4
    ;; float32 _CPU1
    4
    ;; float32 _CPU2
    4
    ;; float32 _CPU3
    4
    ;; float32 _t
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _mode
       (write-long _mode s)
     ;; int32 _duration
       (write-long _duration s)
     ;; int32 _motor1_move
       (write-long _motor1_move s)
     ;; int32 _motor2_move
       (write-long _motor2_move s)
     ;; int32 _motor1_torque
       (write-long _motor1_torque s)
     ;; int32 _motor2_torque
       (write-long _motor2_torque s)
     ;; float32 _PF_cmd
       (sys::poke _PF_cmd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _EV_cmd
       (sys::poke _EV_cmd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _PF_curr
       (sys::poke _PF_curr (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _EV_curr
       (sys::poke _EV_curr (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _CPU0
       (sys::poke _CPU0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _CPU1
       (sys::poke _CPU1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _CPU2
       (sys::poke _CPU2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _CPU3
       (sys::poke _CPU3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _t
       (sys::poke _t (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _duration
     (setq _duration (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _motor1_move
     (setq _motor1_move (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _motor2_move
     (setq _motor2_move (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _motor1_torque
     (setq _motor1_torque (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _motor2_torque
     (setq _motor2_torque (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _PF_cmd
     (setq _PF_cmd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _EV_cmd
     (setq _EV_cmd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _PF_curr
     (setq _PF_curr (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _EV_curr
     (setq _EV_curr (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _CPU0
     (setq _CPU0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _CPU1
     (setq _CPU1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _CPU2
     (setq _CPU2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _CPU3
     (setq _CPU3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _t
     (setq _t (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get tada_ros::MotorDataMsg :md5sum-) "81f4ae7b8853b19df4672cd6f30b2548")
(setf (get tada_ros::MotorDataMsg :datatype-) "tada_ros/MotorDataMsg")
(setf (get tada_ros::MotorDataMsg :definition-)
      "int32 mode
int32 duration
int32 motor1_move
int32 motor2_move
int32 motor1_torque
int32 motor2_torque
float32 PF_cmd
float32 EV_cmd
float32 PF_curr
float32 EV_curr
float32 CPU0
float32 CPU1
float32 CPU2
float32 CPU3
float32 t

")



(provide :tada_ros/MotorDataMsg "81f4ae7b8853b19df4672cd6f30b2548")


