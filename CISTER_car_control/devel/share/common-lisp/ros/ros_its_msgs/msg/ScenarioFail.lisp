; Auto-generated. Do not edit!


(cl:in-package ros_its_msgs-msg)


;//! \htmlinclude ScenarioFail.msg.html

(cl:defclass <ScenarioFail> (roslisp-msg-protocol:ros-message)
  ((car_name
    :reader car_name
    :initarg :car_name
    :type cl:string
    :initform "")
   (FailType
    :reader FailType
    :initarg :FailType
    :type cl:string
    :initform ""))
)

(cl:defclass ScenarioFail (<ScenarioFail>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ScenarioFail>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ScenarioFail)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_its_msgs-msg:<ScenarioFail> is deprecated: use ros_its_msgs-msg:ScenarioFail instead.")))

(cl:ensure-generic-function 'car_name-val :lambda-list '(m))
(cl:defmethod car_name-val ((m <ScenarioFail>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:car_name-val is deprecated.  Use ros_its_msgs-msg:car_name instead.")
  (car_name m))

(cl:ensure-generic-function 'FailType-val :lambda-list '(m))
(cl:defmethod FailType-val ((m <ScenarioFail>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:FailType-val is deprecated.  Use ros_its_msgs-msg:FailType instead.")
  (FailType m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ScenarioFail>) ostream)
  "Serializes a message object of type '<ScenarioFail>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'car_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'car_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'FailType))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'FailType))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ScenarioFail>) istream)
  "Deserializes a message object of type '<ScenarioFail>"
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
      (cl:setf (cl:slot-value msg 'FailType) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'FailType) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ScenarioFail>)))
  "Returns string type for a message object of type '<ScenarioFail>"
  "ros_its_msgs/ScenarioFail")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ScenarioFail)))
  "Returns string type for a message object of type 'ScenarioFail"
  "ros_its_msgs/ScenarioFail")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ScenarioFail>)))
  "Returns md5sum for a message object of type '<ScenarioFail>"
  "e62d5c88cdc7df31630e13cbde053d43")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ScenarioFail)))
  "Returns md5sum for a message object of type 'ScenarioFail"
  "e62d5c88cdc7df31630e13cbde053d43")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ScenarioFail>)))
  "Returns full string definition for message of type '<ScenarioFail>"
  (cl:format cl:nil "~%string car_name~%~%string FailType~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ScenarioFail)))
  "Returns full string definition for message of type 'ScenarioFail"
  (cl:format cl:nil "~%string car_name~%~%string FailType~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ScenarioFail>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'car_name))
     4 (cl:length (cl:slot-value msg 'FailType))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ScenarioFail>))
  "Converts a ROS message object to a list"
  (cl:list 'ScenarioFail
    (cl:cons ':car_name (car_name msg))
    (cl:cons ':FailType (FailType msg))
))
