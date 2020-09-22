;; Auto-generated. Do not edit!


(when (boundp 'image_processing::error_control)
  (if (not (find-package "IMAGE_PROCESSING"))
    (make-package "IMAGE_PROCESSING"))
  (shadow 'error_control (find-package "IMAGE_PROCESSING")))
(unless (find-package "IMAGE_PROCESSING::ERROR_CONTROL")
  (make-package "IMAGE_PROCESSING::ERROR_CONTROL"))

(in-package "ROS")
;;//! \htmlinclude error_control.msg.html


(defclass image_processing::error_control
  :super ros::object
  :slots (_error_steer _control_error_steer _steer_integral _steer_deriv _pid_error_value _theta_error_value _dist_tv _status ))

(defmethod image_processing::error_control
  (:init
   (&key
    ((:error_steer __error_steer) 0.0)
    ((:control_error_steer __control_error_steer) 0.0)
    ((:steer_integral __steer_integral) 0.0)
    ((:steer_deriv __steer_deriv) 0.0)
    ((:pid_error_value __pid_error_value) 0.0)
    ((:theta_error_value __theta_error_value) 0.0)
    ((:dist_tv __dist_tv) 0.0)
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _error_steer (float __error_steer))
   (setq _control_error_steer (float __control_error_steer))
   (setq _steer_integral (float __steer_integral))
   (setq _steer_deriv (float __steer_deriv))
   (setq _pid_error_value (float __pid_error_value))
   (setq _theta_error_value (float __theta_error_value))
   (setq _dist_tv (float __dist_tv))
   (setq _status (round __status))
   self)
  (:error_steer
   (&optional __error_steer)
   (if __error_steer (setq _error_steer __error_steer)) _error_steer)
  (:control_error_steer
   (&optional __control_error_steer)
   (if __control_error_steer (setq _control_error_steer __control_error_steer)) _control_error_steer)
  (:steer_integral
   (&optional __steer_integral)
   (if __steer_integral (setq _steer_integral __steer_integral)) _steer_integral)
  (:steer_deriv
   (&optional __steer_deriv)
   (if __steer_deriv (setq _steer_deriv __steer_deriv)) _steer_deriv)
  (:pid_error_value
   (&optional __pid_error_value)
   (if __pid_error_value (setq _pid_error_value __pid_error_value)) _pid_error_value)
  (:theta_error_value
   (&optional __theta_error_value)
   (if __theta_error_value (setq _theta_error_value __theta_error_value)) _theta_error_value)
  (:dist_tv
   (&optional __dist_tv)
   (if __dist_tv (setq _dist_tv __dist_tv)) _dist_tv)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; float32 _error_steer
    4
    ;; float32 _control_error_steer
    4
    ;; float32 _steer_integral
    4
    ;; float32 _steer_deriv
    4
    ;; float32 _pid_error_value
    4
    ;; float32 _theta_error_value
    4
    ;; float32 _dist_tv
    4
    ;; int32 _status
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _error_steer
       (sys::poke _error_steer (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _control_error_steer
       (sys::poke _control_error_steer (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _steer_integral
       (sys::poke _steer_integral (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _steer_deriv
       (sys::poke _steer_deriv (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pid_error_value
       (sys::poke _pid_error_value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta_error_value
       (sys::poke _theta_error_value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dist_tv
       (sys::poke _dist_tv (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _status
       (write-long _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _error_steer
     (setq _error_steer (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _control_error_steer
     (setq _control_error_steer (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _steer_integral
     (setq _steer_integral (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _steer_deriv
     (setq _steer_deriv (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pid_error_value
     (setq _pid_error_value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta_error_value
     (setq _theta_error_value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dist_tv
     (setq _dist_tv (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _status
     (setq _status (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get image_processing::error_control :md5sum-) "1d6a944f0c56f70ef6f19b82ff1f68dd")
(setf (get image_processing::error_control :datatype-) "image_processing/error_control")
(setf (get image_processing::error_control :definition-)
      "float32 error_steer
float32 control_error_steer
float32 steer_integral
float32 steer_deriv
float32 pid_error_value
float32 theta_error_value
float32 dist_tv
int32 status

")



(provide :image_processing/error_control "1d6a944f0c56f70ef6f19b82ff1f68dd")


