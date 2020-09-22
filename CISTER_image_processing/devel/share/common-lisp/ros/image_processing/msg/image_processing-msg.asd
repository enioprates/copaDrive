
(cl:in-package :asdf)

(defsystem "image_processing-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AckermannDrive" :depends-on ("_package_AckermannDrive"))
    (:file "_package_AckermannDrive" :depends-on ("_package"))
    (:file "AckermannDriveStamped" :depends-on ("_package_AckermannDriveStamped"))
    (:file "_package_AckermannDriveStamped" :depends-on ("_package"))
    (:file "coords" :depends-on ("_package_coords"))
    (:file "_package_coords" :depends-on ("_package"))
    (:file "coords (copy)" :depends-on ("_package_coords (copy)"))
    (:file "_package_coords (copy)" :depends-on ("_package"))
    (:file "drive_param" :depends-on ("_package_drive_param"))
    (:file "_package_drive_param" :depends-on ("_package"))
    (:file "error_control" :depends-on ("_package_error_control"))
    (:file "_package_error_control" :depends-on ("_package"))
  ))