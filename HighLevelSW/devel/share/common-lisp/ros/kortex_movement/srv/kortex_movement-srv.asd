
(cl:in-package :asdf)

(defsystem "kortex_movement-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "cartesian_movement" :depends-on ("_package_cartesian_movement"))
    (:file "_package_cartesian_movement" :depends-on ("_package"))
  ))