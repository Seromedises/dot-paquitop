;; Auto-generated. Do not edit!


(when (boundp 'kortex_movement::cartesian_movement)
  (if (not (find-package "KORTEX_MOVEMENT"))
    (make-package "KORTEX_MOVEMENT"))
  (shadow 'cartesian_movement (find-package "KORTEX_MOVEMENT")))
(unless (find-package "KORTEX_MOVEMENT::CARTESIAN_MOVEMENT")
  (make-package "KORTEX_MOVEMENT::CARTESIAN_MOVEMENT"))
(unless (find-package "KORTEX_MOVEMENT::CARTESIAN_MOVEMENTREQUEST")
  (make-package "KORTEX_MOVEMENT::CARTESIAN_MOVEMENTREQUEST"))
(unless (find-package "KORTEX_MOVEMENT::CARTESIAN_MOVEMENTRESPONSE")
  (make-package "KORTEX_MOVEMENT::CARTESIAN_MOVEMENTRESPONSE"))

(in-package "ROS")





(defclass kortex_movement::cartesian_movementRequest
  :super ros::object
  :slots (_x _y _z _thetax _thetay _thetaz ))

(defmethod kortex_movement::cartesian_movementRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:thetax __thetax) 0.0)
    ((:thetay __thetay) 0.0)
    ((:thetaz __thetaz) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _thetax (float __thetax))
   (setq _thetay (float __thetay))
   (setq _thetaz (float __thetaz))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:thetax
   (&optional __thetax)
   (if __thetax (setq _thetax __thetax)) _thetax)
  (:thetay
   (&optional __thetay)
   (if __thetay (setq _thetay __thetay)) _thetay)
  (:thetaz
   (&optional __thetaz)
   (if __thetaz (setq _thetaz __thetaz)) _thetaz)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ;; float32 _thetax
    4
    ;; float32 _thetay
    4
    ;; float32 _thetaz
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _thetax
       (sys::poke _thetax (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _thetay
       (sys::poke _thetay (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _thetaz
       (sys::poke _thetaz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _thetax
     (setq _thetax (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _thetay
     (setq _thetay (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _thetaz
     (setq _thetaz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass kortex_movement::cartesian_movementResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_movement::cartesian_movementResponse
  (:init
   (&key
    ((:output __output) nil)
    )
   (send-super :init)
   (setq _output __output)
   self)
  (:output
   (&optional __output)
   (if __output (setq _output __output)) _output)
  (:serialization-length
   ()
   (+
    ;; bool _output
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _output
       (if _output (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _output
     (setq _output (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass kortex_movement::cartesian_movement
  :super ros::object
  :slots ())

(setf (get kortex_movement::cartesian_movement :md5sum-) "527d545043520e6207cc8929405c4e9e")
(setf (get kortex_movement::cartesian_movement :datatype-) "kortex_movement/cartesian_movement")
(setf (get kortex_movement::cartesian_movement :request) kortex_movement::cartesian_movementRequest)
(setf (get kortex_movement::cartesian_movement :response) kortex_movement::cartesian_movementResponse)

(defmethod kortex_movement::cartesian_movementRequest
  (:response () (instance kortex_movement::cartesian_movementResponse :init)))

(setf (get kortex_movement::cartesian_movementRequest :md5sum-) "527d545043520e6207cc8929405c4e9e")
(setf (get kortex_movement::cartesian_movementRequest :datatype-) "kortex_movement/cartesian_movementRequest")
(setf (get kortex_movement::cartesian_movementRequest :definition-)
      "float32 x
float32 y
float32 z
float32 thetax
float32 thetay
float32 thetaz
---
bool output

")

(setf (get kortex_movement::cartesian_movementResponse :md5sum-) "527d545043520e6207cc8929405c4e9e")
(setf (get kortex_movement::cartesian_movementResponse :datatype-) "kortex_movement/cartesian_movementResponse")
(setf (get kortex_movement::cartesian_movementResponse :definition-)
      "float32 x
float32 y
float32 z
float32 thetax
float32 thetay
float32 thetaz
---
bool output

")



(provide :kortex_movement/cartesian_movement "527d545043520e6207cc8929405c4e9e")


