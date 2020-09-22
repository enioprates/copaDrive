; Auto-generated. Do not edit!


(cl:in-package image_processing-msg)


;//! \htmlinclude coords.msg.html

(cl:defclass <coords> (roslisp-msg-protocol:ros-message)
  ((X2
    :reader X2
    :initarg :X2
    :type cl:float
    :initform 0.0)
   (X1
    :reader X1
    :initarg :X1
    :type cl:float
    :initform 0.0)
   (dif_X
    :reader dif_X
    :initarg :dif_X
    :type cl:float
    :initform 0.0)
   (Y2
    :reader Y2
    :initarg :Y2
    :type cl:float
    :initform 0.0)
   (Y1
    :reader Y1
    :initarg :Y1
    :type cl:float
    :initform 0.0)
   (dif_Y
    :reader dif_Y
    :initarg :dif_Y
    :type cl:float
    :initform 0.0)
   (slope
    :reader slope
    :initarg :slope
    :type cl:float
    :initform 0.0)
   (length
    :reader length
    :initarg :length
    :type cl:float
    :initform 0.0)
   (intercept
    :reader intercept
    :initarg :intercept
    :type cl:float
    :initform 0.0))
)

(cl:defclass coords (<coords>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <coords>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'coords)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_processing-msg:<coords> is deprecated: use image_processing-msg:coords instead.")))

(cl:ensure-generic-function 'X2-val :lambda-list '(m))
(cl:defmethod X2-val ((m <coords>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing-msg:X2-val is deprecated.  Use image_processing-msg:X2 instead.")
  (X2 m))

(cl:ensure-generic-function 'X1-val :lambda-list '(m))
(cl:defmethod X1-val ((m <coords>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing-msg:X1-val is deprecated.  Use image_processing-msg:X1 instead.")
  (X1 m))

(cl:ensure-generic-function 'dif_X-val :lambda-list '(m))
(cl:defmethod dif_X-val ((m <coords>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing-msg:dif_X-val is deprecated.  Use image_processing-msg:dif_X instead.")
  (dif_X m))

(cl:ensure-generic-function 'Y2-val :lambda-list '(m))
(cl:defmethod Y2-val ((m <coords>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing-msg:Y2-val is deprecated.  Use image_processing-msg:Y2 instead.")
  (Y2 m))

(cl:ensure-generic-function 'Y1-val :lambda-list '(m))
(cl:defmethod Y1-val ((m <coords>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing-msg:Y1-val is deprecated.  Use image_processing-msg:Y1 instead.")
  (Y1 m))

(cl:ensure-generic-function 'dif_Y-val :lambda-list '(m))
(cl:defmethod dif_Y-val ((m <coords>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing-msg:dif_Y-val is deprecated.  Use image_processing-msg:dif_Y instead.")
  (dif_Y m))

(cl:ensure-generic-function 'slope-val :lambda-list '(m))
(cl:defmethod slope-val ((m <coords>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing-msg:slope-val is deprecated.  Use image_processing-msg:slope instead.")
  (slope m))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <coords>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing-msg:length-val is deprecated.  Use image_processing-msg:length instead.")
  (length m))

(cl:ensure-generic-function 'intercept-val :lambda-list '(m))
(cl:defmethod intercept-val ((m <coords>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing-msg:intercept-val is deprecated.  Use image_processing-msg:intercept instead.")
  (intercept m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <coords>) ostream)
  "Serializes a message object of type '<coords>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'X2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'X1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dif_X))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Y2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Y1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dif_Y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'slope))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'intercept))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <coords>) istream)
  "Deserializes a message object of type '<coords>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'X2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'X1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dif_X) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Y2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Y1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dif_Y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'slope) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'length) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'intercept) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<coords>)))
  "Returns string type for a message object of type '<coords>"
  "image_processing/coords")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'coords)))
  "Returns string type for a message object of type 'coords"
  "image_processing/coords")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<coords>)))
  "Returns md5sum for a message object of type '<coords>"
  "07271e516a6d3af46ff39e3801eabeb2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'coords)))
  "Returns md5sum for a message object of type 'coords"
  "07271e516a6d3af46ff39e3801eabeb2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<coords>)))
  "Returns full string definition for message of type '<coords>"
  (cl:format cl:nil "float32 X2~%float32 X1~%float32 dif_X~%float32 Y2~%float32 Y1~%float32 dif_Y~%float32 slope~%float32 length~%float32 intercept~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'coords)))
  "Returns full string definition for message of type 'coords"
  (cl:format cl:nil "float32 X2~%float32 X1~%float32 dif_X~%float32 Y2~%float32 Y1~%float32 dif_Y~%float32 slope~%float32 length~%float32 intercept~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <coords>))
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
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <coords>))
  "Converts a ROS message object to a list"
  (cl:list 'coords
    (cl:cons ':X2 (X2 msg))
    (cl:cons ':X1 (X1 msg))
    (cl:cons ':dif_X (dif_X msg))
    (cl:cons ':Y2 (Y2 msg))
    (cl:cons ':Y1 (Y1 msg))
    (cl:cons ':dif_Y (dif_Y msg))
    (cl:cons ':slope (slope msg))
    (cl:cons ':length (length msg))
    (cl:cons ':intercept (intercept msg))
))
