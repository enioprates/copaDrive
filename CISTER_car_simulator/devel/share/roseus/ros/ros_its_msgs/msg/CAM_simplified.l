;; Auto-generated. Do not edit!


(when (boundp 'ros_its_msgs::CAM_simplified)
  (if (not (find-package "ROS_ITS_MSGS"))
    (make-package "ROS_ITS_MSGS"))
  (shadow 'CAM_simplified (find-package "ROS_ITS_MSGS")))
(unless (find-package "ROS_ITS_MSGS::CAM_SIMPLIFIED")
  (make-package "ROS_ITS_MSGS::CAM_SIMPLIFIED"))

(in-package "ROS")
;;//! \htmlinclude CAM_simplified.msg.html


(defclass ros_its_msgs::CAM_simplified
  :super ros::object
  :slots (_car_name _Station_ID _latitude _longitude _altitude_altitudeValue _heading_headingValue _speed_speedValue _driveDirection _steeringWheelAngle_steeringWheelAngleValue _gasPedalPercent_Value _brakePedalPercent_Value ))

(defmethod ros_its_msgs::CAM_simplified
  (:init
   (&key
    ((:car_name __car_name) "")
    ((:Station_ID __Station_ID) "")
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:altitude_altitudeValue __altitude_altitudeValue) 0.0)
    ((:heading_headingValue __heading_headingValue) 0.0)
    ((:speed_speedValue __speed_speedValue) 0.0)
    ((:driveDirection __driveDirection) 0)
    ((:steeringWheelAngle_steeringWheelAngleValue __steeringWheelAngle_steeringWheelAngleValue) 0.0)
    ((:gasPedalPercent_Value __gasPedalPercent_Value) 0.0)
    ((:brakePedalPercent_Value __brakePedalPercent_Value) 0.0)
    )
   (send-super :init)
   (setq _car_name (string __car_name))
   (setq _Station_ID (string __Station_ID))
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _altitude_altitudeValue (float __altitude_altitudeValue))
   (setq _heading_headingValue (float __heading_headingValue))
   (setq _speed_speedValue (float __speed_speedValue))
   (setq _driveDirection (round __driveDirection))
   (setq _steeringWheelAngle_steeringWheelAngleValue (float __steeringWheelAngle_steeringWheelAngleValue))
   (setq _gasPedalPercent_Value (float __gasPedalPercent_Value))
   (setq _brakePedalPercent_Value (float __brakePedalPercent_Value))
   self)
  (:car_name
   (&optional __car_name)
   (if __car_name (setq _car_name __car_name)) _car_name)
  (:Station_ID
   (&optional __Station_ID)
   (if __Station_ID (setq _Station_ID __Station_ID)) _Station_ID)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:altitude_altitudeValue
   (&optional __altitude_altitudeValue)
   (if __altitude_altitudeValue (setq _altitude_altitudeValue __altitude_altitudeValue)) _altitude_altitudeValue)
  (:heading_headingValue
   (&optional __heading_headingValue)
   (if __heading_headingValue (setq _heading_headingValue __heading_headingValue)) _heading_headingValue)
  (:speed_speedValue
   (&optional __speed_speedValue)
   (if __speed_speedValue (setq _speed_speedValue __speed_speedValue)) _speed_speedValue)
  (:driveDirection
   (&optional __driveDirection)
   (if __driveDirection (setq _driveDirection __driveDirection)) _driveDirection)
  (:steeringWheelAngle_steeringWheelAngleValue
   (&optional __steeringWheelAngle_steeringWheelAngleValue)
   (if __steeringWheelAngle_steeringWheelAngleValue (setq _steeringWheelAngle_steeringWheelAngleValue __steeringWheelAngle_steeringWheelAngleValue)) _steeringWheelAngle_steeringWheelAngleValue)
  (:gasPedalPercent_Value
   (&optional __gasPedalPercent_Value)
   (if __gasPedalPercent_Value (setq _gasPedalPercent_Value __gasPedalPercent_Value)) _gasPedalPercent_Value)
  (:brakePedalPercent_Value
   (&optional __brakePedalPercent_Value)
   (if __brakePedalPercent_Value (setq _brakePedalPercent_Value __brakePedalPercent_Value)) _brakePedalPercent_Value)
  (:serialization-length
   ()
   (+
    ;; string _car_name
    4 (length _car_name)
    ;; string _Station_ID
    4 (length _Station_ID)
    ;; float32 _latitude
    4
    ;; float64 _longitude
    8
    ;; float32 _altitude_altitudeValue
    4
    ;; float32 _heading_headingValue
    4
    ;; float32 _speed_speedValue
    4
    ;; int8 _driveDirection
    1
    ;; float32 _steeringWheelAngle_steeringWheelAngleValue
    4
    ;; float32 _gasPedalPercent_Value
    4
    ;; float32 _brakePedalPercent_Value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _car_name
       (write-long (length _car_name) s) (princ _car_name s)
     ;; string _Station_ID
       (write-long (length _Station_ID) s) (princ _Station_ID s)
     ;; float32 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _altitude_altitudeValue
       (sys::poke _altitude_altitudeValue (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading_headingValue
       (sys::poke _heading_headingValue (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed_speedValue
       (sys::poke _speed_speedValue (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int8 _driveDirection
       (write-byte _driveDirection s)
     ;; float32 _steeringWheelAngle_steeringWheelAngleValue
       (sys::poke _steeringWheelAngle_steeringWheelAngleValue (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gasPedalPercent_Value
       (sys::poke _gasPedalPercent_Value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _brakePedalPercent_Value
       (sys::poke _brakePedalPercent_Value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _car_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _car_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _Station_ID
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Station_ID (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _latitude
     (setq _latitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _altitude_altitudeValue
     (setq _altitude_altitudeValue (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading_headingValue
     (setq _heading_headingValue (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed_speedValue
     (setq _speed_speedValue (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int8 _driveDirection
     (setq _driveDirection (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _driveDirection 127) (setq _driveDirection (- _driveDirection 256)))
   ;; float32 _steeringWheelAngle_steeringWheelAngleValue
     (setq _steeringWheelAngle_steeringWheelAngleValue (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gasPedalPercent_Value
     (setq _gasPedalPercent_Value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _brakePedalPercent_Value
     (setq _brakePedalPercent_Value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ros_its_msgs::CAM_simplified :md5sum-) "5ad8a1ab31db5a9f774bad039fc1ce70")
(setf (get ros_its_msgs::CAM_simplified :datatype-) "ros_its_msgs/CAM_simplified")
(setf (get ros_its_msgs::CAM_simplified :definition-)
      "# IDs

string car_name
string Station_ID

# Reference Position
#int32 latitude
#int64 longitude
#int32 altitude_altitudeValue

float32 latitude
float64 longitude
float32 altitude_altitudeValue

# BasicVehicleContainerHighFrequency (Simplified)

#uint16 heading_headingValue
#uint16 speed_speedValue
float32 heading_headingValue
float32 speed_speedValue

int8 driveDirection

#int16 steeringWheelAngle_steeringWheelAngleValue
float32 steeringWheelAngle_steeringWheelAngleValue

float32 gasPedalPercent_Value
float32 brakePedalPercent_Value

#float32 Ref_distance
#float32 leader_distance






")



(provide :ros_its_msgs/CAM_simplified "5ad8a1ab31db5a9f774bad039fc1ce70")


