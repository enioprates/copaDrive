; Auto-generated. Do not edit!


(cl:in-package ros_its_msgs-msg)


;//! \htmlinclude platoon_dist.msg.html

(cl:defclass <platoon_dist> (roslisp-msg-protocol:ros-message)
  ((car_name
    :reader car_name
    :initarg :car_name
    :type cl:string
    :initform "")
   (Ref_distance
    :reader Ref_distance
    :initarg :Ref_distance
    :type cl:float
    :initform 0.0)
   (leader_distance
    :reader leader_distance
    :initarg :leader_distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass platoon_dist (<platoon_dist>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <platoon_dist>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'platoon_dist)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_its_msgs-msg:<platoon_dist> is deprecated: use ros_its_msgs-msg:platoon_dist instead.")))

(cl:ensure-generic-function 'car_name-val :lambda-list '(m))
(cl:defmethod car_name-val ((m <platoon_dist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:car_name-val is deprecated.  Use ros_its_msgs-msg:car_name instead.")
  (car_name m))

(cl:ensure-generic-function 'Ref_distance-val :lambda-list '(m))
(cl:defmethod Ref_distance-val ((m <platoon_dist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:Ref_distance-val is deprecated.  Use ros_its_msgs-msg:Ref_distance instead.")
  (Ref_distance m))

(cl:ensure-generic-function 'leader_distance-val :lambda-list '(m))
(cl:defmethod leader_distance-val ((m <platoon_dist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:leader_distance-val is deprecated.  Use ros_its_msgs-msg:leader_distance instead.")
  (leader_distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <platoon_dist>) ostream)
  "Serializes a message object of type '<platoon_dist>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'car_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'car_name))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Ref_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'leader_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <platoon_dist>) istream)
  "Deserializes a message object of type '<platoon_dist>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'car_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'car_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Ref_distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'leader_distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<platoon_dist>)))
  "Returns string type for a message object of type '<platoon_dist>"
  "ros_its_msgs/platoon_dist")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'platoon_dist)))
  "Returns string type for a message object of type 'platoon_dist"
  "ros_its_msgs/platoon_dist")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<platoon_dist>)))
  "Returns md5sum for a message object of type '<platoon_dist>"
  "865334224e65761f8e48bcb6f774f442")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'platoon_dist)))
  "Returns md5sum for a message object of type 'platoon_dist"
  "865334224e65761f8e48bcb6f774f442")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<platoon_dist>)))
  "Returns full string definition for message of type '<platoon_dist>"
  (cl:format cl:nil "string car_name~%~%float32 Ref_distance~%float32 leader_distance~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'platoon_dist)))
  "Returns full string definition for message of type 'platoon_dist"
  (cl:format cl:nil "string car_name~%~%float32 Ref_distance~%float32 leader_distance~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <platoon_dist>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'car_name))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <platoon_dist>))
  "Converts a ROS message object to a list"
  (cl:list 'platoon_dist
    (cl:cons ':car_name (car_name msg))
    (cl:cons ':Ref_distance (Ref_distance msg))
    (cl:cons ':leader_distance (leader_distance msg))
))
