; Auto-generated. Do not edit!


(cl:in-package ros_its_msgs-msg)


;//! \htmlinclude Sonar.msg.html

(cl:defclass <Sonar> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (radiation_type
    :reader radiation_type
    :initarg :radiation_type
    :type cl:fixnum
    :initform 0)
   (field_of_view
    :reader field_of_view
    :initarg :field_of_view
    :type cl:float
    :initform 0.0)
   (min_range
    :reader min_range
    :initarg :min_range
    :type cl:float
    :initform 0.0)
   (max_range
    :reader max_range
    :initarg :max_range
    :type cl:float
    :initform 0.0)
   (range
    :reader range
    :initarg :range
    :type cl:float
    :initform 0.0))
)

(cl:defclass Sonar (<Sonar>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Sonar>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Sonar)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_its_msgs-msg:<Sonar> is deprecated: use ros_its_msgs-msg:Sonar instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Sonar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:header-val is deprecated.  Use ros_its_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'radiation_type-val :lambda-list '(m))
(cl:defmethod radiation_type-val ((m <Sonar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:radiation_type-val is deprecated.  Use ros_its_msgs-msg:radiation_type instead.")
  (radiation_type m))

(cl:ensure-generic-function 'field_of_view-val :lambda-list '(m))
(cl:defmethod field_of_view-val ((m <Sonar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:field_of_view-val is deprecated.  Use ros_its_msgs-msg:field_of_view instead.")
  (field_of_view m))

(cl:ensure-generic-function 'min_range-val :lambda-list '(m))
(cl:defmethod min_range-val ((m <Sonar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:min_range-val is deprecated.  Use ros_its_msgs-msg:min_range instead.")
  (min_range m))

(cl:ensure-generic-function 'max_range-val :lambda-list '(m))
(cl:defmethod max_range-val ((m <Sonar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:max_range-val is deprecated.  Use ros_its_msgs-msg:max_range instead.")
  (max_range m))

(cl:ensure-generic-function 'range-val :lambda-list '(m))
(cl:defmethod range-val ((m <Sonar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:range-val is deprecated.  Use ros_its_msgs-msg:range instead.")
  (range m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Sonar>)))
    "Constants for message type '<Sonar>"
  '((:ULTRASOUND . 0)
    (:INFRARED . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Sonar)))
    "Constants for message type 'Sonar"
  '((:ULTRASOUND . 0)
    (:INFRARED . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Sonar>) ostream)
  "Serializes a message object of type '<Sonar>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'radiation_type)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'field_of_view))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min_range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Sonar>) istream)
  "Deserializes a message object of type '<Sonar>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'radiation_type)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'field_of_view) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_range) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_range) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'range) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Sonar>)))
  "Returns string type for a message object of type '<Sonar>"
  "ros_its_msgs/Sonar")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sonar)))
  "Returns string type for a message object of type 'Sonar"
  "ros_its_msgs/Sonar")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Sonar>)))
  "Returns md5sum for a message object of type '<Sonar>"
  "c005c34273dc426c67a020a87bc24148")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Sonar)))
  "Returns md5sum for a message object of type 'Sonar"
  "c005c34273dc426c67a020a87bc24148")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Sonar>)))
  "Returns full string definition for message of type '<Sonar>"
  (cl:format cl:nil "uint8 ULTRASOUND=0~%uint8 INFRARED=1~%std_msgs/Header header~%uint8 radiation_type~%float32 field_of_view~%float32 min_range~%float32 max_range~%float32 range~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Sonar)))
  "Returns full string definition for message of type 'Sonar"
  (cl:format cl:nil "uint8 ULTRASOUND=0~%uint8 INFRARED=1~%std_msgs/Header header~%uint8 radiation_type~%float32 field_of_view~%float32 min_range~%float32 max_range~%float32 range~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Sonar>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Sonar>))
  "Converts a ROS message object to a list"
  (cl:list 'Sonar
    (cl:cons ':header (header msg))
    (cl:cons ':radiation_type (radiation_type msg))
    (cl:cons ':field_of_view (field_of_view msg))
    (cl:cons ':min_range (min_range msg))
    (cl:cons ':max_range (max_range msg))
    (cl:cons ':range (range msg))
))
