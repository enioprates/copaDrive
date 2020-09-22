;; Auto-generated. Do not edit!


(when (boundp 'image_processing::coords)
  (if (not (find-package "IMAGE_PROCESSING"))
    (make-package "IMAGE_PROCESSING"))
  (shadow 'coords (find-package "IMAGE_PROCESSING")))
(unless (find-package "IMAGE_PROCESSING::COORDS")
  (make-package "IMAGE_PROCESSING::COORDS"))

(in-package "ROS")
;;//! \htmlinclude coords.msg.html


(defclass image_processing::coords
  :super ros::object
  :slots (_X2 _X1 _dif_X _Y2 _Y1 _dif_Y _slope _length _intercept ))

(defmethod image_processing::coords
  (:init
   (&key
    ((:X2 __X2) 0.0)
    ((:X1 __X1) 0.0)
    ((:dif_X __dif_X) 0.0)
    ((:Y2 __Y2) 0.0)
    ((:Y1 __Y1) 0.0)
    ((:dif_Y __dif_Y) 0.0)
    ((:slope __slope) 0.0)
    ((:length __length) 0.0)
    ((:intercept __intercept) 0.0)
    )
   (send-super :init)
   (setq _X2 (float __X2))
   (setq _X1 (float __X1))
   (setq _dif_X (float __dif_X))
   (setq _Y2 (float __Y2))
   (setq _Y1 (float __Y1))
   (setq _dif_Y (float __dif_Y))
   (setq _slope (float __slope))
   (setq _length (float __length))
   (setq _intercept (float __intercept))
   self)
  (:X2
   (&optional __X2)
   (if __X2 (setq _X2 __X2)) _X2)
  (:X1
   (&optional __X1)
   (if __X1 (setq _X1 __X1)) _X1)
  (:dif_X
   (&optional __dif_X)
   (if __dif_X (setq _dif_X __dif_X)) _dif_X)
  (:Y2
   (&optional __Y2)
   (if __Y2 (setq _Y2 __Y2)) _Y2)
  (:Y1
   (&optional __Y1)
   (if __Y1 (setq _Y1 __Y1)) _Y1)
  (:dif_Y
   (&optional __dif_Y)
   (if __dif_Y (setq _dif_Y __dif_Y)) _dif_Y)
  (:slope
   (&optional __slope)
   (if __slope (setq _slope __slope)) _slope)
  (:length
   (&optional __length)
   (if __length (setq _length __length)) _length)
  (:intercept
   (&optional __intercept)
   (if __intercept (setq _intercept __intercept)) _intercept)
  (:serialization-length
   ()
   (+
    ;; float32 _X2
    4
    ;; float32 _X1
    4
    ;; float32 _dif_X
    4
    ;; float32 _Y2
    4
    ;; float32 _Y1
    4
    ;; float32 _dif_Y
    4
    ;; float32 _slope
    4
    ;; float32 _length
    4
    ;; float32 _intercept
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _X2
       (sys::poke _X2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _X1
       (sys::poke _X1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dif_X
       (sys::poke _dif_X (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Y2
       (sys::poke _Y2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Y1
       (sys::poke _Y1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dif_Y
       (sys::poke _dif_Y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _slope
       (sys::poke _slope (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _length
       (sys::poke _length (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _intercept
       (sys::poke _intercept (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _X2
     (setq _X2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _X1
     (setq _X1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dif_X
     (setq _dif_X (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Y2
     (setq _Y2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Y1
     (setq _Y1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dif_Y
     (setq _dif_Y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _slope
     (setq _slope (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _length
     (setq _length (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _intercept
     (setq _intercept (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get image_processing::coords :md5sum-) "07271e516a6d3af46ff39e3801eabeb2")
(setf (get image_processing::coords :datatype-) "image_processing/coords")
(setf (get image_processing::coords :definition-)
      "float32 X2
float32 X1
float32 dif_X
float32 Y2
float32 Y1
float32 dif_Y
float32 slope
float32 length
float32 intercept

")



(provide :image_processing/coords "07271e516a6d3af46ff39e3801eabeb2")


