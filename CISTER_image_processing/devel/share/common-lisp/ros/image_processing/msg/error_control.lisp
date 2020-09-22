; Auto-generated. Do not edit!


(cl:in-package image_processing-msg)


;//! \htmlinclude error_control.msg.html

(cl:defclass <error_control> (roslisp-msg-protocol:ros-message)
  ((error_steer
    :reader error_steer
    :initarg :error_steer
    :type cl:float
    :initform 0.0)
   (control_error_steer
    :reader control_error_steer
    :initarg :control_error_steer
    :type cl:float
    :initform 0.0)
   (steer_integral
    :reader steer_integral
    :initarg :steer_integral
    :type cl:float
    :initform 0.0)
   (steer_deriv
    :reader steer_deriv
    :initarg :steer_deriv
    :type cl:float
    :initform 0.0)
   (pid_error_value
    :reader pid_error_value
    :initarg :pid_error_value
    :type cl:float
    :initform 0.0)
   (theta_error_value
    :reader theta_error_value
    :initarg :theta_error_value
    :type cl:float
    :initform 0.0)
   (dist_tv
    :reader dist_tv
    :initarg :dist_tv
    :type cl:float
    :initform 0.0)
   (status
    :reader status
    :initarg :status
    :type cl:integer
    :initform 0))
)

(cl:defclass error_control (<error_control>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <error_control>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'error_control)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_processing-msg:<error_control> is deprecated: use image_processing-msg:error_control instead.")))

(cl:ensure-generic-function 'error_steer-val :lambda-list '(m))
(cl:defmethod error_steer-val ((m <error_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing-msg:error_steer-val is deprecated.  Use image_processing-msg:error_steer instead.")
  (error_steer m))

(cl:ensure-generic-function 'control_error_steer-val :lambda-list '(m))
(cl:defmethod control_error_steer-val ((m <error_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing-msg:control_error_steer-val is deprecated.  Use image_processing-msg:control_error_steer instead.")
  (control_error_steer m))

(cl:ensure-generic-function 'steer_integral-val :lambda-list '(m))
(cl:defmethod steer_integral-val ((m <error_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing-msg:steer_integral-val is deprecated.  Use image_processing-msg:steer_integral instead.")
  (steer_integral m))

(cl:ensure-generic-function 'steer_deriv-val :lambda-list '(m))
(cl:defmethod steer_deriv-val ((m <error_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing-msg:steer_deriv-val is deprecated.  Use image_processing-msg:steer_deriv instead.")
  (steer_deriv m))

(cl:ensure-generic-function 'pid_error_value-val :lambda-list '(m))
(cl:defmethod pid_error_value-val ((m <error_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing-msg:pid_error_value-val is deprecated.  Use image_processing-msg:pid_error_value instead.")
  (pid_error_value m))

(cl:ensure-generic-function 'theta_error_value-val :lambda-list '(m))
(cl:defmethod theta_error_value-val ((m <error_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing-msg:theta_error_value-val is deprecated.  Use image_processing-msg:theta_error_value instead.")
  (theta_error_value m))

(cl:ensure-generic-function 'dist_tv-val :lambda-list '(m))
(cl:defmethod dist_tv-val ((m <error_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing-msg:dist_tv-val is deprecated.  Use image_processing-msg:dist_tv instead.")
  (dist_tv m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <error_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing-msg:status-val is deprecated.  Use image_processing-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <error_control>) ostream)
  "Serializes a message object of type '<error_control>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'error_steer))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'control_error_steer))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steer_integral))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steer_deriv))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pid_error_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta_error_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dist_tv))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <error_control>) istream)
  "Deserializes a message object of type '<error_control>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error_steer) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'control_error_steer) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steer_integral) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steer_deriv) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pid_error_value) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_error_value) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dist_tv) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<error_control>)))
  "Returns string type for a message object of type '<error_control>"
  "image_processing/error_control")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'error_control)))
  "Returns string type for a message object of type 'error_control"
  "image_processing/error_control")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<error_control>)))
  "Returns md5sum for a message object of type '<error_control>"
  "1d6a944f0c56f70ef6f19b82ff1f68dd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'error_control)))
  "Returns md5sum for a message object of type 'error_control"
  "1d6a944f0c56f70ef6f19b82ff1f68dd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<error_control>)))
  "Returns full string definition for message of type '<error_control>"
  (cl:format cl:nil "float32 error_steer~%float32 control_error_steer~%float32 steer_integral~%float32 steer_deriv~%float32 pid_error_value~%float32 theta_error_value~%float32 dist_tv~%int32 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'error_control)))
  "Returns full string definition for message of type 'error_control"
  (cl:format cl:nil "float32 error_steer~%float32 control_error_steer~%float32 steer_integral~%float32 steer_deriv~%float32 pid_error_value~%float32 theta_error_value~%float32 dist_tv~%int32 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <error_control>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <error_control>))
  "Converts a ROS message object to a list"
  (cl:list 'error_control
    (cl:cons ':error_steer (error_steer msg))
    (cl:cons ':control_error_steer (control_error_steer msg))
    (cl:cons ':steer_integral (steer_integral msg))
    (cl:cons ':steer_deriv (steer_deriv msg))
    (cl:cons ':pid_error_value (pid_error_value msg))
    (cl:cons ':theta_error_value (theta_error_value msg))
    (cl:cons ':dist_tv (dist_tv msg))
    (cl:cons ':status (status msg))
))
