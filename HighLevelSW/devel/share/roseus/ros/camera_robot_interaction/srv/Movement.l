;; Auto-generated. Do not edit!


(when (boundp 'camera_robot_interaction::Movement)
  (if (not (find-package "CAMERA_ROBOT_INTERACTION"))
    (make-package "CAMERA_ROBOT_INTERACTION"))
  (shadow 'Movement (find-package "CAMERA_ROBOT_INTERACTION")))
(unless (find-package "CAMERA_ROBOT_INTERACTION::MOVEMENT")
  (make-package "CAMERA_ROBOT_INTERACTION::MOVEMENT"))
(unless (find-package "CAMERA_ROBOT_INTERACTION::MOVEMENTREQUEST")
  (make-package "CAMERA_ROBOT_INTERACTION::MOVEMENTREQUEST"))
(unless (find-package "CAMERA_ROBOT_INTERACTION::MOVEMENTRESPONSE")
  (make-package "CAMERA_ROBOT_INTERACTION::MOVEMENTRESPONSE"))

(in-package "ROS")





(defclass camera_robot_interaction::MovementRequest
  :super ros::object
  :slots (_r11 _r12 _r13 _r14 _r21 _r22 _r23 _r24 _r31 _r32 _r33 _r34 _gripper _rotate _rest_pos ))

(defmethod camera_robot_interaction::MovementRequest
  (:init
   (&key
    ((:r11 __r11) 0.0)
    ((:r12 __r12) 0.0)
    ((:r13 __r13) 0.0)
    ((:r14 __r14) 0.0)
    ((:r21 __r21) 0.0)
    ((:r22 __r22) 0.0)
    ((:r23 __r23) 0.0)
    ((:r24 __r24) 0.0)
    ((:r31 __r31) 0.0)
    ((:r32 __r32) 0.0)
    ((:r33 __r33) 0.0)
    ((:r34 __r34) 0.0)
    ((:gripper __gripper) 0.0)
    ((:rotate __rotate) nil)
    ((:rest_pos __rest_pos) nil)
    )
   (send-super :init)
   (setq _r11 (float __r11))
   (setq _r12 (float __r12))
   (setq _r13 (float __r13))
   (setq _r14 (float __r14))
   (setq _r21 (float __r21))
   (setq _r22 (float __r22))
   (setq _r23 (float __r23))
   (setq _r24 (float __r24))
   (setq _r31 (float __r31))
   (setq _r32 (float __r32))
   (setq _r33 (float __r33))
   (setq _r34 (float __r34))
   (setq _gripper (float __gripper))
   (setq _rotate __rotate)
   (setq _rest_pos __rest_pos)
   self)
  (:r11
   (&optional __r11)
   (if __r11 (setq _r11 __r11)) _r11)
  (:r12
   (&optional __r12)
   (if __r12 (setq _r12 __r12)) _r12)
  (:r13
   (&optional __r13)
   (if __r13 (setq _r13 __r13)) _r13)
  (:r14
   (&optional __r14)
   (if __r14 (setq _r14 __r14)) _r14)
  (:r21
   (&optional __r21)
   (if __r21 (setq _r21 __r21)) _r21)
  (:r22
   (&optional __r22)
   (if __r22 (setq _r22 __r22)) _r22)
  (:r23
   (&optional __r23)
   (if __r23 (setq _r23 __r23)) _r23)
  (:r24
   (&optional __r24)
   (if __r24 (setq _r24 __r24)) _r24)
  (:r31
   (&optional __r31)
   (if __r31 (setq _r31 __r31)) _r31)
  (:r32
   (&optional __r32)
   (if __r32 (setq _r32 __r32)) _r32)
  (:r33
   (&optional __r33)
   (if __r33 (setq _r33 __r33)) _r33)
  (:r34
   (&optional __r34)
   (if __r34 (setq _r34 __r34)) _r34)
  (:gripper
   (&optional __gripper)
   (if __gripper (setq _gripper __gripper)) _gripper)
  (:rotate
   (&optional __rotate)
   (if __rotate (setq _rotate __rotate)) _rotate)
  (:rest_pos
   (&optional __rest_pos)
   (if __rest_pos (setq _rest_pos __rest_pos)) _rest_pos)
  (:serialization-length
   ()
   (+
    ;; float32 _r11
    4
    ;; float32 _r12
    4
    ;; float32 _r13
    4
    ;; float32 _r14
    4
    ;; float32 _r21
    4
    ;; float32 _r22
    4
    ;; float32 _r23
    4
    ;; float32 _r24
    4
    ;; float32 _r31
    4
    ;; float32 _r32
    4
    ;; float32 _r33
    4
    ;; float32 _r34
    4
    ;; float32 _gripper
    4
    ;; bool _rotate
    1
    ;; bool _rest_pos
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _r11
       (sys::poke _r11 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _r12
       (sys::poke _r12 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _r13
       (sys::poke _r13 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _r14
       (sys::poke _r14 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _r21
       (sys::poke _r21 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _r22
       (sys::poke _r22 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _r23
       (sys::poke _r23 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _r24
       (sys::poke _r24 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _r31
       (sys::poke _r31 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _r32
       (sys::poke _r32 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _r33
       (sys::poke _r33 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _r34
       (sys::poke _r34 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gripper
       (sys::poke _gripper (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _rotate
       (if _rotate (write-byte -1 s) (write-byte 0 s))
     ;; bool _rest_pos
       (if _rest_pos (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _r11
     (setq _r11 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _r12
     (setq _r12 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _r13
     (setq _r13 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _r14
     (setq _r14 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _r21
     (setq _r21 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _r22
     (setq _r22 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _r23
     (setq _r23 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _r24
     (setq _r24 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _r31
     (setq _r31 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _r32
     (setq _r32 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _r33
     (setq _r33 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _r34
     (setq _r34 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gripper
     (setq _gripper (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _rotate
     (setq _rotate (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _rest_pos
     (setq _rest_pos (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass camera_robot_interaction::MovementResponse
  :super ros::object
  :slots (_Success _gripper_fb ))

(defmethod camera_robot_interaction::MovementResponse
  (:init
   (&key
    ((:Success __Success) 0)
    ((:gripper_fb __gripper_fb) 0.0)
    )
   (send-super :init)
   (setq _Success (round __Success))
   (setq _gripper_fb (float __gripper_fb))
   self)
  (:Success
   (&optional __Success)
   (if __Success (setq _Success __Success)) _Success)
  (:gripper_fb
   (&optional __gripper_fb)
   (if __gripper_fb (setq _gripper_fb __gripper_fb)) _gripper_fb)
  (:serialization-length
   ()
   (+
    ;; int64 _Success
    8
    ;; float32 _gripper_fb
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _Success
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _Success (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _Success) (= (length (_Success . bv)) 2)) ;; bignum
              (write-long (ash (elt (_Success . bv) 0) 0) s)
              (write-long (ash (elt (_Success . bv) 1) -1) s))
             ((and (class _Success) (= (length (_Success . bv)) 1)) ;; big1
              (write-long (elt (_Success . bv) 0) s)
              (write-long (if (>= _Success 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _Success s)(write-long (if (>= _Success 0) 0 #xffffffff) s)))
     ;; float32 _gripper_fb
       (sys::poke _gripper_fb (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _Success
#+(or :alpha :irix6 :x86_64)
      (setf _Success (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _Success (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float32 _gripper_fb
     (setq _gripper_fb (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass camera_robot_interaction::Movement
  :super ros::object
  :slots ())

(setf (get camera_robot_interaction::Movement :md5sum-) "4787e27e3721f1f2004cf17997b3e3ae")
(setf (get camera_robot_interaction::Movement :datatype-) "camera_robot_interaction/Movement")
(setf (get camera_robot_interaction::Movement :request) camera_robot_interaction::MovementRequest)
(setf (get camera_robot_interaction::Movement :response) camera_robot_interaction::MovementResponse)

(defmethod camera_robot_interaction::MovementRequest
  (:response () (instance camera_robot_interaction::MovementResponse :init)))

(setf (get camera_robot_interaction::MovementRequest :md5sum-) "4787e27e3721f1f2004cf17997b3e3ae")
(setf (get camera_robot_interaction::MovementRequest :datatype-) "camera_robot_interaction/MovementRequest")
(setf (get camera_robot_interaction::MovementRequest :definition-)
      "float32 r11
float32 r12
float32 r13
float32 r14
float32 r21
float32 r22
float32 r23
float32 r24
float32 r31
float32 r32
float32 r33
float32 r34
float32 gripper
bool rotate
bool rest_pos
---
int64 Success
float32 gripper_fb

")

(setf (get camera_robot_interaction::MovementResponse :md5sum-) "4787e27e3721f1f2004cf17997b3e3ae")
(setf (get camera_robot_interaction::MovementResponse :datatype-) "camera_robot_interaction/MovementResponse")
(setf (get camera_robot_interaction::MovementResponse :definition-)
      "float32 r11
float32 r12
float32 r13
float32 r14
float32 r21
float32 r22
float32 r23
float32 r24
float32 r31
float32 r32
float32 r33
float32 r34
float32 gripper
bool rotate
bool rest_pos
---
int64 Success
float32 gripper_fb

")



(provide :camera_robot_interaction/Movement "4787e27e3721f1f2004cf17997b3e3ae")


