;; Auto-generated. Do not edit!


(when (boundp 'ros_its_msgs::ScenarioFail)
  (if (not (find-package "ROS_ITS_MSGS"))
    (make-package "ROS_ITS_MSGS"))
  (shadow 'ScenarioFail (find-package "ROS_ITS_MSGS")))
(unless (find-package "ROS_ITS_MSGS::SCENARIOFAIL")
  (make-package "ROS_ITS_MSGS::SCENARIOFAIL"))

(in-package "ROS")
;;//! \htmlinclude ScenarioFail.msg.html


(defclass ros_its_msgs::ScenarioFail
  :super ros::object
  :slots (_car_name _FailType ))

(defmethod ros_its_msgs::ScenarioFail
  (:init
   (&key
    ((:car_name __car_name) "")
    ((:FailType __FailType) "")
    )
   (send-super :init)
   (setq _car_name (string __car_name))
   (setq _FailType (string __FailType))
   self)
  (:car_name
   (&optional __car_name)
   (if __car_name (setq _car_name __car_name)) _car_name)
  (:FailType
   (&optional __FailType)
   (if __FailType (setq _FailType __FailType)) _FailType)
  (:serialization-length
   ()
   (+
    ;; string _car_name
    4 (length _car_name)
    ;; string _FailType
    4 (length _FailType)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _car_name
       (write-long (length _car_name) s) (princ _car_name s)
     ;; string _FailType
       (write-long (length _FailType) s) (princ _FailType s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _car_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _car_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _FailType
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _FailType (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ros_its_msgs::ScenarioFail :md5sum-) "e62d5c88cdc7df31630e13cbde053d43")
(setf (get ros_its_msgs::ScenarioFail :datatype-) "ros_its_msgs/ScenarioFail")
(setf (get ros_its_msgs::ScenarioFail :definition-)
      "
string car_name

string FailType



")



(provide :ros_its_msgs/ScenarioFail "e62d5c88cdc7df31630e13cbde053d43")


