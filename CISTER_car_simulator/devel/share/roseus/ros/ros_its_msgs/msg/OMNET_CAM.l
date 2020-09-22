;; Auto-generated. Do not edit!


(when (boundp 'ros_its_msgs::OMNET_CAM)
  (if (not (find-package "ROS_ITS_MSGS"))
    (make-package "ROS_ITS_MSGS"))
  (shadow 'OMNET_CAM (find-package "ROS_ITS_MSGS")))
(unless (find-package "ROS_ITS_MSGS::OMNET_CAM")
  (make-package "ROS_ITS_MSGS::OMNET_CAM"))

(in-package "ROS")
;;//! \htmlinclude OMNET_CAM.msg.html


(defclass ros_its_msgs::OMNET_CAM
  :super ros::object
  :slots (_car_name _latitude _longitude _altitude _heading _speed _acceleration ))

(defmethod ros_its_msgs::OMNET_CAM
  (:init
   (&key
    ((:car_name __car_name) "")
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:altitude __altitude) 0.0)
    ((:heading __heading) 0.0)
    ((:speed __speed) 0.0)
    ((:acceleration __acceleration) 0.0)
    )
   (send-super :init)
   (setq _car_name (string __car_name))
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _altitude (float __altitude))
   (setq _heading (float __heading))
   (setq _speed (float __speed))
   (setq _acceleration (float __acceleration))
   self)
  (:car_name
   (&optional __car_name)
   (if __car_name (setq _car_name __car_name)) _car_name)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:acceleration
   (&optional __acceleration)
   (if __acceleration (setq _acceleration __acceleration)) _acceleration)
  (:serialization-length
   ()
   (+
    ;; string _car_name
    4 (length _car_name)
    ;; float32 _latitude
    4
    ;; float64 _longitude
    8
    ;; float32 _altitude
    4
    ;; float32 _heading
    4
    ;; float32 _speed
    4
    ;; float32 _acceleration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _car_name
       (write-long (length _car_name) s) (princ _car_name s)
     ;; float32 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading
       (sys::poke _heading (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _acceleration
       (sys::poke _acceleration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _car_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _car_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _latitude
     (setq _latitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _altitude
     (setq _altitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading
     (setq _heading (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _acceleration
     (setq _acceleration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ros_its_msgs::OMNET_CAM :md5sum-) "c8e185aa09a6a67e142adbb8f653456b")
(setf (get ros_its_msgs::OMNET_CAM :datatype-) "ros_its_msgs/OMNET_CAM")
(setf (get ros_its_msgs::OMNET_CAM :definition-)
      "string car_name

float32 latitude
float64 longitude
float32 altitude

float32 heading
float32 speed

float32 acceleration

")



(provide :ros_its_msgs/OMNET_CAM "c8e185aa09a6a67e142adbb8f653456b")


