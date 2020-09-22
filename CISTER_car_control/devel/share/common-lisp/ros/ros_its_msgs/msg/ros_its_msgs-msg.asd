
(cl:in-package :asdf)

(defsystem "ros_its_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CAM_simplified" :depends-on ("_package_CAM_simplified"))
    (:file "_package_CAM_simplified" :depends-on ("_package"))
    (:file "CLWarning" :depends-on ("_package_CLWarning"))
    (:file "_package_CLWarning" :depends-on ("_package"))
    (:file "ScenarioFail" :depends-on ("_package_ScenarioFail"))
    (:file "_package_ScenarioFail" :depends-on ("_package"))
    (:file "platoon_dist" :depends-on ("_package_platoon_dist"))
    (:file "_package_platoon_dist" :depends-on ("_package"))
  ))