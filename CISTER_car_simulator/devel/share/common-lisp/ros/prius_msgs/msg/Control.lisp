; Auto-generated. Do not edit!


(cl:in-package prius_msgs-msg)


;//! \htmlinclude Control.msg.html

(cl:defclass <Control> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (throttle
    :reader throttle
    :initarg :throttle
    :type cl:float
    :initform 0.0)
   (brake
    :reader brake
    :initarg :brake
    :type cl:float
    :initform 0.0)
   (steer
    :reader steer
    :initarg :steer
    :type cl:float
    :initform 0.0)
   (shift_gears
    :reader shift_gears
    :initarg :shift_gears
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Control (<Control>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Control>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Control)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name prius_msgs-msg:<Control> is deprecated: use prius_msgs-msg:Control instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader prius_msgs-msg:header-val is deprecated.  Use prius_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'throttle-val :lambda-list '(m))
(cl:defmethod throttle-val ((m <Control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader prius_msgs-msg:throttle-val is deprecated.  Use prius_msgs-msg:throttle instead.")
  (throttle m))

(cl:ensure-generic-function 'brake-val :lambda-list '(m))
(cl:defmethod brake-val ((m <Control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader prius_msgs-msg:brake-val is deprecated.  Use prius_msgs-msg:brake instead.")
  (brake m))

(cl:ensure-generic-function 'steer-val :lambda-list '(m))
(cl:defmethod steer-val ((m <Control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader prius_msgs-msg:steer-val is deprecated.  Use prius_msgs-msg:steer instead.")
  (steer m))

(cl:ensure-generic-function 'shift_gears-val :lambda-list '(m))
(cl:defmethod shift_gears-val ((m <Control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader prius_msgs-msg:shift_gears-val is deprecated.  Use prius_msgs-msg:shift_gears instead.")
  (shift_gears m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Control>)))
    "Constants for message type '<Control>"
  '((:NO_COMMAND . 0)
    (:NEUTRAL . 1)
    (:FORWARD . 2)
    (:REVERSE . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Control)))
    "Constants for message type 'Control"
  '((:NO_COMMAND . 0)
    (:NEUTRAL . 1)
    (:FORWARD . 2)
    (:REVERSE . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Control>) ostream)
  "Serializes a message object of type '<Control>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'throttle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'brake))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'steer))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'shift_gears)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Control>) istream)
  "Deserializes a message object of type '<Control>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'throttle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'brake) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steer) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'shift_gears)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Control>)))
  "Returns string type for a message object of type '<Control>"
  "prius_msgs/Control")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Control)))
  "Returns string type for a message object of type 'Control"
  "prius_msgs/Control")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Control>)))
  "Returns md5sum for a message object of type '<Control>"
  "643e03b408b8368cf4ae109e399336ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Control)))
  "Returns md5sum for a message object of type 'Control"
  "643e03b408b8368cf4ae109e399336ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Control>)))
  "Returns full string definition for message of type '<Control>"
  (cl:format cl:nil "Header header~%~%# Range 0 to 1, 1 is max throttle~%float64 throttle~%# Range 0 to 1, 1 is max brake~%float64 brake~%# Range -1 to +1, +1 is maximum left turn~%float64 steer~%~%uint8 NO_COMMAND=0~%uint8 NEUTRAL=1~%uint8 FORWARD=2~%uint8 REVERSE=3~%~%uint8 shift_gears~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Control)))
  "Returns full string definition for message of type 'Control"
  (cl:format cl:nil "Header header~%~%# Range 0 to 1, 1 is max throttle~%float64 throttle~%# Range 0 to 1, 1 is max brake~%float64 brake~%# Range -1 to +1, +1 is maximum left turn~%float64 steer~%~%uint8 NO_COMMAND=0~%uint8 NEUTRAL=1~%uint8 FORWARD=2~%uint8 REVERSE=3~%~%uint8 shift_gears~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Control>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Control>))
  "Converts a ROS message object to a list"
  (cl:list 'Control
    (cl:cons ':header (header msg))
    (cl:cons ':throttle (throttle msg))
    (cl:cons ':brake (brake msg))
    (cl:cons ':steer (steer msg))
    (cl:cons ':shift_gears (shift_gears msg))
))
