; Auto-generated. Do not edit!


(cl:in-package ros_its_msgs-msg)


;//! \htmlinclude CAM_simplified.msg.html

(cl:defclass <CAM_simplified> (roslisp-msg-protocol:ros-message)
  ((car_name
    :reader car_name
    :initarg :car_name
    :type cl:string
    :initform "")
   (Station_ID
    :reader Station_ID
    :initarg :Station_ID
    :type cl:string
    :initform "")
   (latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:float
    :initform 0.0)
   (altitude_altitudeValue
    :reader altitude_altitudeValue
    :initarg :altitude_altitudeValue
    :type cl:float
    :initform 0.0)
   (heading_headingValue
    :reader heading_headingValue
    :initarg :heading_headingValue
    :type cl:float
    :initform 0.0)
   (speed_speedValue
    :reader speed_speedValue
    :initarg :speed_speedValue
    :type cl:float
    :initform 0.0)
   (driveDirection
    :reader driveDirection
    :initarg :driveDirection
    :type cl:fixnum
    :initform 0)
   (steeringWheelAngle_steeringWheelAngleValue
    :reader steeringWheelAngle_steeringWheelAngleValue
    :initarg :steeringWheelAngle_steeringWheelAngleValue
    :type cl:float
    :initform 0.0)
   (gasPedalPercent_Value
    :reader gasPedalPercent_Value
    :initarg :gasPedalPercent_Value
    :type cl:float
    :initform 0.0)
   (brakePedalPercent_Value
    :reader brakePedalPercent_Value
    :initarg :brakePedalPercent_Value
    :type cl:float
    :initform 0.0))
)

(cl:defclass CAM_simplified (<CAM_simplified>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CAM_simplified>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CAM_simplified)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_its_msgs-msg:<CAM_simplified> is deprecated: use ros_its_msgs-msg:CAM_simplified instead.")))

(cl:ensure-generic-function 'car_name-val :lambda-list '(m))
(cl:defmethod car_name-val ((m <CAM_simplified>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:car_name-val is deprecated.  Use ros_its_msgs-msg:car_name instead.")
  (car_name m))

(cl:ensure-generic-function 'Station_ID-val :lambda-list '(m))
(cl:defmethod Station_ID-val ((m <CAM_simplified>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:Station_ID-val is deprecated.  Use ros_its_msgs-msg:Station_ID instead.")
  (Station_ID m))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <CAM_simplified>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:latitude-val is deprecated.  Use ros_its_msgs-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <CAM_simplified>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:longitude-val is deprecated.  Use ros_its_msgs-msg:longitude instead.")
  (longitude m))

(cl:ensure-generic-function 'altitude_altitudeValue-val :lambda-list '(m))
(cl:defmethod altitude_altitudeValue-val ((m <CAM_simplified>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:altitude_altitudeValue-val is deprecated.  Use ros_its_msgs-msg:altitude_altitudeValue instead.")
  (altitude_altitudeValue m))

(cl:ensure-generic-function 'heading_headingValue-val :lambda-list '(m))
(cl:defmethod heading_headingValue-val ((m <CAM_simplified>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:heading_headingValue-val is deprecated.  Use ros_its_msgs-msg:heading_headingValue instead.")
  (heading_headingValue m))

(cl:ensure-generic-function 'speed_speedValue-val :lambda-list '(m))
(cl:defmethod speed_speedValue-val ((m <CAM_simplified>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:speed_speedValue-val is deprecated.  Use ros_its_msgs-msg:speed_speedValue instead.")
  (speed_speedValue m))

(cl:ensure-generic-function 'driveDirection-val :lambda-list '(m))
(cl:defmethod driveDirection-val ((m <CAM_simplified>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:driveDirection-val is deprecated.  Use ros_its_msgs-msg:driveDirection instead.")
  (driveDirection m))

(cl:ensure-generic-function 'steeringWheelAngle_steeringWheelAngleValue-val :lambda-list '(m))
(cl:defmethod steeringWheelAngle_steeringWheelAngleValue-val ((m <CAM_simplified>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:steeringWheelAngle_steeringWheelAngleValue-val is deprecated.  Use ros_its_msgs-msg:steeringWheelAngle_steeringWheelAngleValue instead.")
  (steeringWheelAngle_steeringWheelAngleValue m))

(cl:ensure-generic-function 'gasPedalPercent_Value-val :lambda-list '(m))
(cl:defmethod gasPedalPercent_Value-val ((m <CAM_simplified>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:gasPedalPercent_Value-val is deprecated.  Use ros_its_msgs-msg:gasPedalPercent_Value instead.")
  (gasPedalPercent_Value m))

(cl:ensure-generic-function 'brakePedalPercent_Value-val :lambda-list '(m))
(cl:defmethod brakePedalPercent_Value-val ((m <CAM_simplified>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:brakePedalPercent_Value-val is deprecated.  Use ros_its_msgs-msg:brakePedalPercent_Value instead.")
  (brakePedalPercent_Value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CAM_simplified>) ostream)
  "Serializes a message object of type '<CAM_simplified>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'car_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'car_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Station_ID))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Station_ID))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'longitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'altitude_altitudeValue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'heading_headingValue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed_speedValue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'driveDirection)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steeringWheelAngle_steeringWheelAngleValue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gasPedalPercent_Value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'brakePedalPercent_Value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CAM_simplified>) istream)
  "Deserializes a message object of type '<CAM_simplified>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'car_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'car_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Station_ID) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Station_ID) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'altitude_altitudeValue) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_headingValue) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed_speedValue) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'driveDirection) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steeringWheelAngle_steeringWheelAngleValue) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gasPedalPercent_Value) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'brakePedalPercent_Value) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CAM_simplified>)))
  "Returns string type for a message object of type '<CAM_simplified>"
  "ros_its_msgs/CAM_simplified")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CAM_simplified)))
  "Returns string type for a message object of type 'CAM_simplified"
  "ros_its_msgs/CAM_simplified")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CAM_simplified>)))
  "Returns md5sum for a message object of type '<CAM_simplified>"
  "5ad8a1ab31db5a9f774bad039fc1ce70")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CAM_simplified)))
  "Returns md5sum for a message object of type 'CAM_simplified"
  "5ad8a1ab31db5a9f774bad039fc1ce70")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CAM_simplified>)))
  "Returns full string definition for message of type '<CAM_simplified>"
  (cl:format cl:nil "# IDs~%~%string car_name~%string Station_ID~%~%# Reference Position~%#int32 latitude~%#int64 longitude~%#int32 altitude_altitudeValue~%~%float32 latitude~%float64 longitude~%float32 altitude_altitudeValue~%~%# BasicVehicleContainerHighFrequency (Simplified)~%~%#uint16 heading_headingValue~%#uint16 speed_speedValue~%float32 heading_headingValue~%float32 speed_speedValue~%~%int8 driveDirection~%~%#int16 steeringWheelAngle_steeringWheelAngleValue~%float32 steeringWheelAngle_steeringWheelAngleValue~%~%float32 gasPedalPercent_Value~%float32 brakePedalPercent_Value~%~%#float32 Ref_distance~%#float32 leader_distance~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CAM_simplified)))
  "Returns full string definition for message of type 'CAM_simplified"
  (cl:format cl:nil "# IDs~%~%string car_name~%string Station_ID~%~%# Reference Position~%#int32 latitude~%#int64 longitude~%#int32 altitude_altitudeValue~%~%float32 latitude~%float64 longitude~%float32 altitude_altitudeValue~%~%# BasicVehicleContainerHighFrequency (Simplified)~%~%#uint16 heading_headingValue~%#uint16 speed_speedValue~%float32 heading_headingValue~%float32 speed_speedValue~%~%int8 driveDirection~%~%#int16 steeringWheelAngle_steeringWheelAngleValue~%float32 steeringWheelAngle_steeringWheelAngleValue~%~%float32 gasPedalPercent_Value~%float32 brakePedalPercent_Value~%~%#float32 Ref_distance~%#float32 leader_distance~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CAM_simplified>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'car_name))
     4 (cl:length (cl:slot-value msg 'Station_ID))
     4
     8
     4
     4
     4
     1
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CAM_simplified>))
  "Converts a ROS message object to a list"
  (cl:list 'CAM_simplified
    (cl:cons ':car_name (car_name msg))
    (cl:cons ':Station_ID (Station_ID msg))
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
    (cl:cons ':altitude_altitudeValue (altitude_altitudeValue msg))
    (cl:cons ':heading_headingValue (heading_headingValue msg))
    (cl:cons ':speed_speedValue (speed_speedValue msg))
    (cl:cons ':driveDirection (driveDirection msg))
    (cl:cons ':steeringWheelAngle_steeringWheelAngleValue (steeringWheelAngle_steeringWheelAngleValue msg))
    (cl:cons ':gasPedalPercent_Value (gasPedalPercent_Value msg))
    (cl:cons ':brakePedalPercent_Value (brakePedalPercent_Value msg))
))
