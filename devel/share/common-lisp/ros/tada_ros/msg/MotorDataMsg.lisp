; Auto-generated. Do not edit!


(cl:in-package tada_ros-msg)


;//! \htmlinclude MotorDataMsg.msg.html

(cl:defclass <MotorDataMsg> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0)
   (duration
    :reader duration
    :initarg :duration
    :type cl:integer
    :initform 0)
   (motor1_move
    :reader motor1_move
    :initarg :motor1_move
    :type cl:integer
    :initform 0)
   (motor2_move
    :reader motor2_move
    :initarg :motor2_move
    :type cl:integer
    :initform 0)
   (motor1_torque
    :reader motor1_torque
    :initarg :motor1_torque
    :type cl:integer
    :initform 0)
   (motor2_torque
    :reader motor2_torque
    :initarg :motor2_torque
    :type cl:integer
    :initform 0)
   (PF_cmd
    :reader PF_cmd
    :initarg :PF_cmd
    :type cl:float
    :initform 0.0)
   (EV_cmd
    :reader EV_cmd
    :initarg :EV_cmd
    :type cl:float
    :initform 0.0)
   (theta_cmd
    :reader theta_cmd
    :initarg :theta_cmd
    :type cl:float
    :initform 0.0)
   (alpha_cmd
    :reader alpha_cmd
    :initarg :alpha_cmd
    :type cl:float
    :initform 0.0)
   (PF_curr
    :reader PF_curr
    :initarg :PF_curr
    :type cl:float
    :initform 0.0)
   (EV_curr
    :reader EV_curr
    :initarg :EV_curr
    :type cl:float
    :initform 0.0)
   (CPU0
    :reader CPU0
    :initarg :CPU0
    :type cl:float
    :initform 0.0)
   (CPU1
    :reader CPU1
    :initarg :CPU1
    :type cl:float
    :initform 0.0)
   (CPU2
    :reader CPU2
    :initarg :CPU2
    :type cl:float
    :initform 0.0)
   (CPU3
    :reader CPU3
    :initarg :CPU3
    :type cl:float
    :initform 0.0)
   (t
    :reader t
    :initarg :t
    :type cl:float
    :initform 0.0)
   (valid
    :reader valid
    :initarg :valid
    :type cl:float
    :initform 0.0))
)

(cl:defclass MotorDataMsg (<MotorDataMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorDataMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorDataMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tada_ros-msg:<MotorDataMsg> is deprecated: use tada_ros-msg:MotorDataMsg instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <MotorDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:mode-val is deprecated.  Use tada_ros-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <MotorDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:duration-val is deprecated.  Use tada_ros-msg:duration instead.")
  (duration m))

(cl:ensure-generic-function 'motor1_move-val :lambda-list '(m))
(cl:defmethod motor1_move-val ((m <MotorDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:motor1_move-val is deprecated.  Use tada_ros-msg:motor1_move instead.")
  (motor1_move m))

(cl:ensure-generic-function 'motor2_move-val :lambda-list '(m))
(cl:defmethod motor2_move-val ((m <MotorDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:motor2_move-val is deprecated.  Use tada_ros-msg:motor2_move instead.")
  (motor2_move m))

(cl:ensure-generic-function 'motor1_torque-val :lambda-list '(m))
(cl:defmethod motor1_torque-val ((m <MotorDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:motor1_torque-val is deprecated.  Use tada_ros-msg:motor1_torque instead.")
  (motor1_torque m))

(cl:ensure-generic-function 'motor2_torque-val :lambda-list '(m))
(cl:defmethod motor2_torque-val ((m <MotorDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:motor2_torque-val is deprecated.  Use tada_ros-msg:motor2_torque instead.")
  (motor2_torque m))

(cl:ensure-generic-function 'PF_cmd-val :lambda-list '(m))
(cl:defmethod PF_cmd-val ((m <MotorDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:PF_cmd-val is deprecated.  Use tada_ros-msg:PF_cmd instead.")
  (PF_cmd m))

(cl:ensure-generic-function 'EV_cmd-val :lambda-list '(m))
(cl:defmethod EV_cmd-val ((m <MotorDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:EV_cmd-val is deprecated.  Use tada_ros-msg:EV_cmd instead.")
  (EV_cmd m))

(cl:ensure-generic-function 'theta_cmd-val :lambda-list '(m))
(cl:defmethod theta_cmd-val ((m <MotorDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:theta_cmd-val is deprecated.  Use tada_ros-msg:theta_cmd instead.")
  (theta_cmd m))

(cl:ensure-generic-function 'alpha_cmd-val :lambda-list '(m))
(cl:defmethod alpha_cmd-val ((m <MotorDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:alpha_cmd-val is deprecated.  Use tada_ros-msg:alpha_cmd instead.")
  (alpha_cmd m))

(cl:ensure-generic-function 'PF_curr-val :lambda-list '(m))
(cl:defmethod PF_curr-val ((m <MotorDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:PF_curr-val is deprecated.  Use tada_ros-msg:PF_curr instead.")
  (PF_curr m))

(cl:ensure-generic-function 'EV_curr-val :lambda-list '(m))
(cl:defmethod EV_curr-val ((m <MotorDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:EV_curr-val is deprecated.  Use tada_ros-msg:EV_curr instead.")
  (EV_curr m))

(cl:ensure-generic-function 'CPU0-val :lambda-list '(m))
(cl:defmethod CPU0-val ((m <MotorDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:CPU0-val is deprecated.  Use tada_ros-msg:CPU0 instead.")
  (CPU0 m))

(cl:ensure-generic-function 'CPU1-val :lambda-list '(m))
(cl:defmethod CPU1-val ((m <MotorDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:CPU1-val is deprecated.  Use tada_ros-msg:CPU1 instead.")
  (CPU1 m))

(cl:ensure-generic-function 'CPU2-val :lambda-list '(m))
(cl:defmethod CPU2-val ((m <MotorDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:CPU2-val is deprecated.  Use tada_ros-msg:CPU2 instead.")
  (CPU2 m))

(cl:ensure-generic-function 'CPU3-val :lambda-list '(m))
(cl:defmethod CPU3-val ((m <MotorDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:CPU3-val is deprecated.  Use tada_ros-msg:CPU3 instead.")
  (CPU3 m))

(cl:ensure-generic-function 't-val :lambda-list '(m))
(cl:defmethod t-val ((m <MotorDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:t-val is deprecated.  Use tada_ros-msg:t instead.")
  (t m))

(cl:ensure-generic-function 'valid-val :lambda-list '(m))
(cl:defmethod valid-val ((m <MotorDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tada_ros-msg:valid-val is deprecated.  Use tada_ros-msg:valid instead.")
  (valid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorDataMsg>) ostream)
  "Serializes a message object of type '<MotorDataMsg>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'duration)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor1_move)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor2_move)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor1_torque)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor2_torque)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'PF_cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'EV_cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta_cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'alpha_cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'PF_curr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'EV_curr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'CPU0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'CPU1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'CPU2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'CPU3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 't))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'valid))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorDataMsg>) istream)
  "Deserializes a message object of type '<MotorDataMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'duration) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor1_move) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor2_move) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor1_torque) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor2_torque) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'PF_cmd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'EV_cmd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_cmd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'alpha_cmd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'PF_curr) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'EV_curr) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'CPU0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'CPU1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'CPU2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'CPU3) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 't) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'valid) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorDataMsg>)))
  "Returns string type for a message object of type '<MotorDataMsg>"
  "tada_ros/MotorDataMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorDataMsg)))
  "Returns string type for a message object of type 'MotorDataMsg"
  "tada_ros/MotorDataMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorDataMsg>)))
  "Returns md5sum for a message object of type '<MotorDataMsg>"
  "5a25d71eeb893c75765db5f0795e4d73")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorDataMsg)))
  "Returns md5sum for a message object of type 'MotorDataMsg"
  "5a25d71eeb893c75765db5f0795e4d73")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorDataMsg>)))
  "Returns full string definition for message of type '<MotorDataMsg>"
  (cl:format cl:nil "int32 mode~%int32 duration~%int32 motor1_move~%int32 motor2_move~%int32 motor1_torque~%int32 motor2_torque~%float32 PF_cmd~%float32 EV_cmd~%float32 theta_cmd~%float32 alpha_cmd~%float32 PF_curr~%float32 EV_curr~%float32 CPU0~%float32 CPU1~%float32 CPU2~%float32 CPU3~%float32 t~%float32 valid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorDataMsg)))
  "Returns full string definition for message of type 'MotorDataMsg"
  (cl:format cl:nil "int32 mode~%int32 duration~%int32 motor1_move~%int32 motor2_move~%int32 motor1_torque~%int32 motor2_torque~%float32 PF_cmd~%float32 EV_cmd~%float32 theta_cmd~%float32 alpha_cmd~%float32 PF_curr~%float32 EV_curr~%float32 CPU0~%float32 CPU1~%float32 CPU2~%float32 CPU3~%float32 t~%float32 valid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorDataMsg>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorDataMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorDataMsg
    (cl:cons ':mode (mode msg))
    (cl:cons ':duration (duration msg))
    (cl:cons ':motor1_move (motor1_move msg))
    (cl:cons ':motor2_move (motor2_move msg))
    (cl:cons ':motor1_torque (motor1_torque msg))
    (cl:cons ':motor2_torque (motor2_torque msg))
    (cl:cons ':PF_cmd (PF_cmd msg))
    (cl:cons ':EV_cmd (EV_cmd msg))
    (cl:cons ':theta_cmd (theta_cmd msg))
    (cl:cons ':alpha_cmd (alpha_cmd msg))
    (cl:cons ':PF_curr (PF_curr msg))
    (cl:cons ':EV_curr (EV_curr msg))
    (cl:cons ':CPU0 (CPU0 msg))
    (cl:cons ':CPU1 (CPU1 msg))
    (cl:cons ':CPU2 (CPU2 msg))
    (cl:cons ':CPU3 (CPU3 msg))
    (cl:cons ':t (t msg))
    (cl:cons ':valid (valid msg))
))
