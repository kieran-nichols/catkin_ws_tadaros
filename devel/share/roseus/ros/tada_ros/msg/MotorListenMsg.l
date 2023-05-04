;; Auto-generated. Do not edit!


(when (boundp 'tada_ros::MotorListenMsg)
  (if (not (find-package "TADA_ROS"))
    (make-package "TADA_ROS"))
  (shadow 'MotorListenMsg (find-package "TADA_ROS")))
(unless (find-package "TADA_ROS::MOTORLISTENMSG")
  (make-package "TADA_ROS::MOTORLISTENMSG"))

(in-package "ROS")
;;//! \htmlinclude MotorListenMsg.msg.html


(defclass tada_ros::MotorListenMsg
  :super ros::object
  :slots (_curr_pos1 _curr_pos2 _toff _motor_fail _t ))

(defmethod tada_ros::MotorListenMsg
  (:init
   (&key
    ((:curr_pos1 __curr_pos1) 0)
    ((:curr_pos2 __curr_pos2) 0)
    ((:toff __toff) 0)
    ((:motor_fail __motor_fail) 0)
    ((:t __t) 0.0)
    )
   (send-super :init)
   (setq _curr_pos1 (round __curr_pos1))
   (setq _curr_pos2 (round __curr_pos2))
   (setq _toff (round __toff))
   (setq _motor_fail (round __motor_fail))
   (setq _t (float __t))
   self)
  (:curr_pos1
   (&optional __curr_pos1)
   (if __curr_pos1 (setq _curr_pos1 __curr_pos1)) _curr_pos1)
  (:curr_pos2
   (&optional __curr_pos2)
   (if __curr_pos2 (setq _curr_pos2 __curr_pos2)) _curr_pos2)
  (:toff
   (&optional __toff)
   (if __toff (setq _toff __toff)) _toff)
  (:motor_fail
   (&optional __motor_fail)
   (if __motor_fail (setq _motor_fail __motor_fail)) _motor_fail)
  (:t
   (&optional __t)
   (if __t (setq _t __t)) _t)
  (:serialization-length
   ()
   (+
    ;; int32 _curr_pos1
    4
    ;; int32 _curr_pos2
    4
    ;; int64 _toff
    8
    ;; int32 _motor_fail
    4
    ;; float32 _t
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _curr_pos1
       (write-long _curr_pos1 s)
     ;; int32 _curr_pos2
       (write-long _curr_pos2 s)
     ;; int64 _toff
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _toff (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _toff) (= (length (_toff . bv)) 2)) ;; bignum
              (write-long (ash (elt (_toff . bv) 0) 0) s)
              (write-long (ash (elt (_toff . bv) 1) -1) s))
             ((and (class _toff) (= (length (_toff . bv)) 1)) ;; big1
              (write-long (elt (_toff . bv) 0) s)
              (write-long (if (>= _toff 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _toff s)(write-long (if (>= _toff 0) 0 #xffffffff) s)))
     ;; int32 _motor_fail
       (write-long _motor_fail s)
     ;; float32 _t
       (sys::poke _t (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _curr_pos1
     (setq _curr_pos1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _curr_pos2
     (setq _curr_pos2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int64 _toff
#+(or :alpha :irix6 :x86_64)
      (setf _toff (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _toff (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int32 _motor_fail
     (setq _motor_fail (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _t
     (setq _t (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get tada_ros::MotorListenMsg :md5sum-) "16758636abd34f69fe5a976e7e2a04ca")
(setf (get tada_ros::MotorListenMsg :datatype-) "tada_ros/MotorListenMsg")
(setf (get tada_ros::MotorListenMsg :definition-)
      "int32 curr_pos1
int32 curr_pos2
int64 toff
int32 motor_fail
float32 t
")



(provide :tada_ros/MotorListenMsg "16758636abd34f69fe5a976e7e2a04ca")


