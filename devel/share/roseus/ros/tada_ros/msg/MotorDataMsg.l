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
  :slots (_mode _duration _motor1_move _motor2_move _motor1_torque _motor2_torque _PF _EV _t ))

(defmethod tada_ros::MotorDataMsg
  (:init
   (&key
    ((:mode __mode) 0)
    ((:duration __duration) 0)
    ((:motor1_move __motor1_move) 0)
    ((:motor2_move __motor2_move) 0)
    ((:motor1_torque __motor1_torque) 0)
    ((:motor2_torque __motor2_torque) 0)
    ((:PF __PF) 0.0)
    ((:EV __EV) 0.0)
    ((:t __t) 0.0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _duration (round __duration))
   (setq _motor1_move (round __motor1_move))
   (setq _motor2_move (round __motor2_move))
   (setq _motor1_torque (round __motor1_torque))
   (setq _motor2_torque (round __motor2_torque))
   (setq _PF (float __PF))
   (setq _EV (float __EV))
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
  (:PF
   (&optional __PF)
   (if __PF (setq _PF __PF)) _PF)
  (:EV
   (&optional __EV)
   (if __EV (setq _EV __EV)) _EV)
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
    ;; float32 _PF
    4
    ;; float32 _EV
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
     ;; float32 _PF
       (sys::poke _PF (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _EV
       (sys::poke _EV (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32 _PF
     (setq _PF (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _EV
     (setq _EV (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _t
     (setq _t (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get tada_ros::MotorDataMsg :md5sum-) "0122060c184091d374c522c731386ece")
(setf (get tada_ros::MotorDataMsg :datatype-) "tada_ros/MotorDataMsg")
(setf (get tada_ros::MotorDataMsg :definition-)
      "int32 mode
int32 duration
int32 motor1_move
int32 motor2_move
int32 motor1_torque
int32 motor2_torque
float32 PF
float32 EV
float32 t

")



(provide :tada_ros/MotorDataMsg "0122060c184091d374c522c731386ece")


