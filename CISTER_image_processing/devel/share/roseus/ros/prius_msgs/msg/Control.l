;; Auto-generated. Do not edit!


(when (boundp 'prius_msgs::Control)
  (if (not (find-package "PRIUS_MSGS"))
    (make-package "PRIUS_MSGS"))
  (shadow 'Control (find-package "PRIUS_MSGS")))
(unless (find-package "PRIUS_MSGS::CONTROL")
  (make-package "PRIUS_MSGS::CONTROL"))

(in-package "ROS")
;;//! \htmlinclude Control.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*NO_COMMAND*" (find-package "PRIUS_MSGS::CONTROL"))
(shadow '*NO_COMMAND* (find-package "PRIUS_MSGS::CONTROL"))
(defconstant prius_msgs::Control::*NO_COMMAND* 0)
(intern "*NEUTRAL*" (find-package "PRIUS_MSGS::CONTROL"))
(shadow '*NEUTRAL* (find-package "PRIUS_MSGS::CONTROL"))
(defconstant prius_msgs::Control::*NEUTRAL* 1)
(intern "*FORWARD*" (find-package "PRIUS_MSGS::CONTROL"))
(shadow '*FORWARD* (find-package "PRIUS_MSGS::CONTROL"))
(defconstant prius_msgs::Control::*FORWARD* 2)
(intern "*REVERSE*" (find-package "PRIUS_MSGS::CONTROL"))
(shadow '*REVERSE* (find-package "PRIUS_MSGS::CONTROL"))
(defconstant prius_msgs::Control::*REVERSE* 3)
(defclass prius_msgs::Control
  :super ros::object
  :slots (_header _throttle _brake _steer _shift_gears ))

(defmethod prius_msgs::Control
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:throttle __throttle) 0.0)
    ((:brake __brake) 0.0)
    ((:steer __steer) 0.0)
    ((:shift_gears __shift_gears) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _throttle (float __throttle))
   (setq _brake (float __brake))
   (setq _steer (float __steer))
   (setq _shift_gears (round __shift_gears))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:throttle
   (&optional __throttle)
   (if __throttle (setq _throttle __throttle)) _throttle)
  (:brake
   (&optional __brake)
   (if __brake (setq _brake __brake)) _brake)
  (:steer
   (&optional __steer)
   (if __steer (setq _steer __steer)) _steer)
  (:shift_gears
   (&optional __shift_gears)
   (if __shift_gears (setq _shift_gears __shift_gears)) _shift_gears)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _throttle
    8
    ;; float64 _brake
    8
    ;; float64 _steer
    8
    ;; uint8 _shift_gears
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _throttle
       (sys::poke _throttle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _brake
       (sys::poke _brake (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _steer
       (sys::poke _steer (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _shift_gears
       (write-byte _shift_gears s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _throttle
     (setq _throttle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _brake
     (setq _brake (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _steer
     (setq _steer (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _shift_gears
     (setq _shift_gears (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get prius_msgs::Control :md5sum-) "643e03b408b8368cf4ae109e399336ec")
(setf (get prius_msgs::Control :datatype-) "prius_msgs/Control")
(setf (get prius_msgs::Control :definition-)
      "Header header

# Range 0 to 1, 1 is max throttle
float64 throttle
# Range 0 to 1, 1 is max brake
float64 brake
# Range -1 to +1, +1 is maximum left turn
float64 steer

uint8 NO_COMMAND=0
uint8 NEUTRAL=1
uint8 FORWARD=2
uint8 REVERSE=3

uint8 shift_gears

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :prius_msgs/Control "643e03b408b8368cf4ae109e399336ec")


