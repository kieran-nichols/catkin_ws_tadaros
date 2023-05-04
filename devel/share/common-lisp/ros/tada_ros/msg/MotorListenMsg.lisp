; Auto-generated. Do not edit!


(cl:in-package tada_ros-msg)


;//! \htmlinclude MotorListenMsg.msg.html

(cl:defclass <MotorListenMsg> (roslisp-msg-protocol:ros-message)
  ((curr_pos1
    :reader curr_pos1
    :initarg :curr_pos1
    :type cl:integer
    :initform 0)
   (curr_pos2
    :reader curr_pos2
    :initarg :curr_pos2
    :type cl:integer
    :initform 0)
   (toff
    :reader toff
    :initarg :toff
    :type cl:integer
    :initform 0)
   (motor_fail
    :reader motor_fail
    :initarg :motor_fail
    :type cl:integer
    :initform 0)
   (t
    :reader t
    :initarg :t
    :type cl:float
    :initform 0.0))
)

(cl:defclass MotorListenMsg (<MotorListenMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorListenMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorListenMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tada_ros-msg:<MotorListenMsg> is deprecated: use tada_ros-msg:MotorListenMsg instead.")))

(cl:ensure-generic-function 'curr_pos1-val :lambda-list '(m))
(cl:defmethod curr_pos1-val ((m <MotorListenMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:curr_pos1-val is deprecated.  Use tada_ros-msg:curr_pos1 instead.")
  (curr_pos1 m))

(cl:ensure-generic-function 'curr_pos2-val :lambda-list '(m))
(cl:defmethod curr_pos2-val ((m <MotorListenMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:curr_pos2-val is deprecated.  Use tada_ros-msg:curr_pos2 instead.")
  (curr_pos2 m))

(cl:ensure-generic-function 'toff-val :lambda-list '(m))
(cl:defmethod toff-val ((m <MotorListenMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:toff-val is deprecated.  Use tada_ros-msg:toff instead.")
  (toff m))

(cl:ensure-generic-function 'motor_fail-val :lambda-list '(m))
(cl:defmethod motor_fail-val ((m <MotorListenMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:motor_fail-val is deprecated.  Use tada_ros-msg:motor_fail instead.")
  (motor_fail m))

(cl:ensure-generic-function 't-val :lambda-list '(m))
(cl:defmethod t-val ((m <MotorListenMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:t-val is deprecated.  Use tada_ros-msg:t instead.")
  (t m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorListenMsg>) ostream)
  "Serializes a message object of type '<MotorListenMsg>"
  (cl:let* ((signed (cl:slot-value msg 'curr_pos1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'curr_pos2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'toff)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor_fail)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 't))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorListenMsg>) istream)
  "Deserializes a message object of type '<MotorListenMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'curr_pos1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'curr_pos2) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'toff) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor_fail) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 't) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorListenMsg>)))
  "Returns string type for a message object of type '<MotorListenMsg>"
  "tada_ros/MotorListenMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorListenMsg)))
  "Returns string type for a message object of type 'MotorListenMsg"
  "tada_ros/MotorListenMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorListenMsg>)))
  "Returns md5sum for a message object of type '<MotorListenMsg>"
  "16758636abd34f69fe5a976e7e2a04ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorListenMsg)))
  "Returns md5sum for a message object of type 'MotorListenMsg"
  "16758636abd34f69fe5a976e7e2a04ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorListenMsg>)))
  "Returns full string definition for message of type '<MotorListenMsg>"
  (cl:format cl:nil "int32 curr_pos1~%int32 curr_pos2~%int64 toff~%int32 motor_fail~%float32 t~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorListenMsg)))
  "Returns full string definition for message of type 'MotorListenMsg"
  (cl:format cl:nil "int32 curr_pos1~%int32 curr_pos2~%int64 toff~%int32 motor_fail~%float32 t~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorListenMsg>))
  (cl:+ 0
     4
     4
     8
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorListenMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorListenMsg
    (cl:cons ':curr_pos1 (curr_pos1 msg))
    (cl:cons ':curr_pos2 (curr_pos2 msg))
    (cl:cons ':toff (toff msg))
    (cl:cons ':motor_fail (motor_fail msg))
    (cl:cons ':t (t msg))
))
