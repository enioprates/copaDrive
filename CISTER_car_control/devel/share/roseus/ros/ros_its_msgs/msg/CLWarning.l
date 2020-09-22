;; Auto-generated. Do not edit!


(when (boundp 'ros_its_msgs::CLWarning)
  (if (not (find-package "ROS_ITS_MSGS"))
    (make-package "ROS_ITS_MSGS"))
  (shadow 'CLWarning (find-package "ROS_ITS_MSGS")))
(unless (find-package "ROS_ITS_MSGS::CLWARNING")
  (make-package "ROS_ITS_MSGS::CLWARNING"))

(in-package "ROS")
;;//! \htmlinclude CLWarning.msg.html


(defclass ros_its_msgs::CLWarning
  :super ros::object
  :slots (_car_name _CLWType ))

(defmethod ros_its_msgs::CLWarning
  (:init
   (&key
    ((:car_name __car_name) "")
    ((:CLWType __CLWType) "")
    )
   (send-super :init)
   (setq _car_name (string __car_name))
   (setq _CLWType (string __CLWType))
   self)
  (:car_name
   (&optional __car_name)
   (if __car_name (setq _car_name __car_name)) _car_name)
  (:CLWType
   (&optional __CLWType)
   (if __CLWType (setq _CLWType __CLWType)) _CLWType)
  (:serialization-length
   ()
   (+
    ;; string _car_name
    4 (length _car_name)
    ;; string _CLWType
    4 (length _CLWType)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _car_name
       (write-long (length _car_name) s) (princ _car_name s)
     ;; string _CLWType
       (write-long (length _CLWType) s) (princ _CLWType s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _car_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _car_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _CLWType
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _CLWType (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ros_its_msgs::CLWarning :md5sum-) "a3e52a7449922f3c43207fb5cac4c802")
(setf (get ros_its_msgs::CLWarning :datatype-) "ros_its_msgs/CLWarning")
(setf (get ros_its_msgs::CLWarning :definition-)
      "string car_name

string CLWType

")



(provide :ros_its_msgs/CLWarning "a3e52a7449922f3c43207fb5cac4c802")


