
(cl:in-package :asdf)

(defsystem "camera_robot_interaction-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Movement" :depends-on ("_package_Movement"))
    (:file "_package_Movement" :depends-on ("_package"))
  ))