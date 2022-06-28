
(cl:in-package :asdf)

(defsystem "kortex_tools-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SaveData" :depends-on ("_package_SaveData"))
    (:file "_package_SaveData" :depends-on ("_package"))
  ))