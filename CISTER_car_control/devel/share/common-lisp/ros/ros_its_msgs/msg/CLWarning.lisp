; Auto-generated. Do not edit!


(cl:in-package ros_its_msgs-msg)


;//! \htmlinclude CLWarning.msg.html

(cl:defclass <CLWarning> (roslisp-msg-protocol:ros-message)
  ((car_name
    :reader car_name
    :initarg :car_name
    :type cl:string
    :initform "")
   (CLWType
    :reader CLWType
    :initarg :CLWType
    :type cl:string
    :initform ""))
)

(cl:defclass CLWarning (<CLWarning>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CLWarning>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CLWarning)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_its_msgs-msg:<CLWarning> is deprecated: use ros_its_msgs-msg:CLWarning instead.")))

(cl:ensure-generic-function 'car_name-val :lambda-list '(m))
(cl:defmethod car_name-val ((m <CLWarning>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:car_name-val is deprecated.  Use ros_its_msgs-msg:car_name instead.")
  (car_name m))

(cl:ensure-generic-function 'CLWType-val :lambda-list '(m))
(cl:defmethod CLWType-val ((m <CLWarning>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_its_msgs-msg:CLWType-val is deprecated.  Use ros_its_msgs-msg:CLWType instead.")
  (CLWType m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CLWarning>) ostream)
  "Serializes a message object of type '<CLWarning>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'car_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'car_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'CLWType))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'CLWType))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CLWarning>) istream)
  "Deserializes a message object of type '<CLWarning>"
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
      (cl:setf (cl:slot-value msg 'CLWType) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'CLWType) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CLWarning>)))
  "Returns string type for a message object of type '<CLWarning>"
  "ros_its_msgs/CLWarning")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CLWarning)))
  "Returns string type for a message object of type 'CLWarning"
  "ros_its_msgs/CLWarning")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CLWarning>)))
  "Returns md5sum for a message object of type '<CLWarning>"
  "a3e52a7449922f3c43207fb5cac4c802")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CLWarning)))
  "Returns md5sum for a message object of type 'CLWarning"
  "a3e52a7449922f3c43207fb5cac4c802")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CLWarning>)))
  "Returns full string definition for message of type '<CLWarning>"
  (cl:format cl:nil "string car_name~%~%string CLWType~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CLWarning)))
  "Returns full string definition for message of type 'CLWarning"
  (cl:format cl:nil "string car_name~%~%string CLWType~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CLWarning>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'car_name))
     4 (cl:length (cl:slot-value msg 'CLWType))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CLWarning>))
  "Converts a ROS message object to a list"
  (cl:list 'CLWarning
    (cl:cons ':car_name (car_name msg))
    (cl:cons ':CLWType (CLWType msg))
))
