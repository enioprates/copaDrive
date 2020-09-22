;; Auto-generated. Do not edit!


(when (boundp 'ros_its_msgs::Sonar)
  (if (not (find-package "ROS_ITS_MSGS"))
    (make-package "ROS_ITS_MSGS"))
  (shadow 'Sonar (find-package "ROS_ITS_MSGS")))
(unless (find-package "ROS_ITS_MSGS::SONAR")
  (make-package "ROS_ITS_MSGS::SONAR"))

(in-package "ROS")
;;//! \htmlinclude Sonar.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*ULTRASOUND*" (find-package "ROS_ITS_MSGS::SONAR"))
(shadow '*ULTRASOUND* (find-package "ROS_ITS_MSGS::SONAR"))
(defconstant ros_its_msgs::Sonar::*ULTRASOUND* 0)
(intern "*INFRARED*" (find-package "ROS_ITS_MSGS::SONAR"))
(shadow '*INFRARED* (find-package "ROS_ITS_MSGS::SONAR"))
(defconstant ros_its_msgs::Sonar::*INFRARED* 1)
(defclass ros_its_msgs::Sonar
  :super ros::object
  :slots (_header _radiation_type _field_of_view _min_range _max_range _range ))

(defmethod ros_its_msgs::Sonar
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:radiation_type __radiation_type) 0)
    ((:field_of_view __field_of_view) 0.0)
    ((:min_range __min_range) 0.0)
    ((:max_range __max_range) 0.0)
    ((:range __range) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _radiation_type (round __radiation_type))
   (setq _field_of_view (float __field_of_view))
   (setq _min_range (float __min_range))
   (setq _max_range (float __max_range))
   (setq _range (float __range))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:radiation_type
   (&optional __radiation_type)
   (if __radiation_type (setq _radiation_type __radiation_type)) _radiation_type)
  (:field_of_view
   (&optional __field_of_view)
   (if __field_of_view (setq _field_of_view __field_of_view)) _field_of_view)
  (:min_range
   (&optional __min_range)
   (if __min_range (setq _min_range __min_range)) _min_range)
  (:max_range
   (&optional __max_range)
   (if __max_range (setq _max_range __max_range)) _max_range)
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _radiation_type
    1
    ;; float32 _field_of_view
    4
    ;; float32 _min_range
    4
    ;; float32 _max_range
    4
    ;; float32 _range
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _radiation_type
       (write-byte _radiation_type s)
     ;; float32 _field_of_view
       (sys::poke _field_of_view (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _min_range
       (sys::poke _min_range (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_range
       (sys::poke _max_range (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _range
       (sys::poke _range (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _radiation_type
     (setq _radiation_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _field_of_view
     (setq _field_of_view (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _min_range
     (setq _min_range (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_range
     (setq _max_range (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _range
     (setq _range (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ros_its_msgs::Sonar :md5sum-) "c005c34273dc426c67a020a87bc24148")
(setf (get ros_its_msgs::Sonar :datatype-) "ros_its_msgs/Sonar")
(setf (get ros_its_msgs::Sonar :definition-)
      "uint8 ULTRASOUND=0
uint8 INFRARED=1
std_msgs/Header header
uint8 radiation_type
float32 field_of_view
float32 min_range
float32 max_range
float32 range
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



(provide :ros_its_msgs/Sonar "c005c34273dc426c67a020a87bc24148")


