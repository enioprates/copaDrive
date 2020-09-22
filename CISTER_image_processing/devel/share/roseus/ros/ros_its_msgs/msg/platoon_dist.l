;; Auto-generated. Do not edit!


(when (boundp 'ros_its_msgs::platoon_dist)
  (if (not (find-package "ROS_ITS_MSGS"))
    (make-package "ROS_ITS_MSGS"))
  (shadow 'platoon_dist (find-package "ROS_ITS_MSGS")))
(unless (find-package "ROS_ITS_MSGS::PLATOON_DIST")
  (make-package "ROS_ITS_MSGS::PLATOON_DIST"))

(in-package "ROS")
;;//! \htmlinclude platoon_dist.msg.html


(defclass ros_its_msgs::platoon_dist
  :super ros::object
  :slots (_car_name _Ref_distance _leader_distance ))

(defmethod ros_its_msgs::platoon_dist
  (:init
   (&key
    ((:car_name __car_name) "")
    ((:Ref_distance __Ref_distance) 0.0)
    ((:leader_distance __leader_distance) 0.0)
    )
   (send-super :init)
   (setq _car_name (string __car_name))
   (setq _Ref_distance (float __Ref_distance))
   (setq _leader_distance (float __leader_distance))
   self)
  (:car_name
   (&optional __car_name)
   (if __car_name (setq _car_name __car_name)) _car_name)
  (:Ref_distance
   (&optional __Ref_distance)
   (if __Ref_distance (setq _Ref_distance __Ref_distance)) _Ref_distance)
  (:leader_distance
   (&optional __leader_distance)
   (if __leader_distance (setq _leader_distance __leader_distance)) _leader_distance)
  (:serialization-length
   ()
   (+
    ;; string _car_name
    4 (length _car_name)
    ;; float32 _Ref_distance
    4
    ;; float32 _leader_distance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _car_name
       (write-long (length _car_name) s) (princ _car_name s)
     ;; float32 _Ref_distance
       (sys::poke _Ref_distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _leader_distance
       (sys::poke _leader_distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _car_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _car_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _Ref_distance
     (setq _Ref_distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _leader_distance
     (setq _leader_distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ros_its_msgs::platoon_dist :md5sum-) "865334224e65761f8e48bcb6f774f442")
(setf (get ros_its_msgs::platoon_dist :datatype-) "ros_its_msgs/platoon_dist")
(setf (get ros_its_msgs::platoon_dist :definition-)
      "string car_name

float32 Ref_distance
float32 leader_distance


")



(provide :ros_its_msgs/platoon_dist "865334224e65761f8e48bcb6f774f442")


