;; Auto-generated. Do not edit!


(when (boundp 'kortex_tools::SaveData)
  (if (not (find-package "KORTEX_TOOLS"))
    (make-package "KORTEX_TOOLS"))
  (shadow 'SaveData (find-package "KORTEX_TOOLS")))
(unless (find-package "KORTEX_TOOLS::SAVEDATA")
  (make-package "KORTEX_TOOLS::SAVEDATA"))
(unless (find-package "KORTEX_TOOLS::SAVEDATAREQUEST")
  (make-package "KORTEX_TOOLS::SAVEDATAREQUEST"))
(unless (find-package "KORTEX_TOOLS::SAVEDATARESPONSE")
  (make-package "KORTEX_TOOLS::SAVEDATARESPONSE"))

(in-package "ROS")





(defclass kortex_tools::SaveDataRequest
  :super ros::object
  :slots (_Save ))

(defmethod kortex_tools::SaveDataRequest
  (:init
   (&key
    ((:Save __Save) 0)
    )
   (send-super :init)
   (setq _Save (round __Save))
   self)
  (:Save
   (&optional __Save)
   (if __Save (setq _Save __Save)) _Save)
  (:serialization-length
   ()
   (+
    ;; int64 _Save
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _Save
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _Save (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _Save) (= (length (_Save . bv)) 2)) ;; bignum
              (write-long (ash (elt (_Save . bv) 0) 0) s)
              (write-long (ash (elt (_Save . bv) 1) -1) s))
             ((and (class _Save) (= (length (_Save . bv)) 1)) ;; big1
              (write-long (elt (_Save . bv) 0) s)
              (write-long (if (>= _Save 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _Save s)(write-long (if (>= _Save 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _Save
#+(or :alpha :irix6 :x86_64)
      (setf _Save (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _Save (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass kortex_tools::SaveDataResponse
  :super ros::object
  :slots (_Success ))

(defmethod kortex_tools::SaveDataResponse
  (:init
   (&key
    ((:Success __Success) 0)
    )
   (send-super :init)
   (setq _Success (round __Success))
   self)
  (:Success
   (&optional __Success)
   (if __Success (setq _Success __Success)) _Success)
  (:serialization-length
   ()
   (+
    ;; int64 _Success
    8
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
   ;;
   self)
  )

(defclass kortex_tools::SaveData
  :super ros::object
  :slots ())

(setf (get kortex_tools::SaveData :md5sum-) "1497dd37643804fd87f9722c47ea18be")
(setf (get kortex_tools::SaveData :datatype-) "kortex_tools/SaveData")
(setf (get kortex_tools::SaveData :request) kortex_tools::SaveDataRequest)
(setf (get kortex_tools::SaveData :response) kortex_tools::SaveDataResponse)

(defmethod kortex_tools::SaveDataRequest
  (:response () (instance kortex_tools::SaveDataResponse :init)))

(setf (get kortex_tools::SaveDataRequest :md5sum-) "1497dd37643804fd87f9722c47ea18be")
(setf (get kortex_tools::SaveDataRequest :datatype-) "kortex_tools/SaveDataRequest")
(setf (get kortex_tools::SaveDataRequest :definition-)
      "int64 Save
---
int64 Success

")

(setf (get kortex_tools::SaveDataResponse :md5sum-) "1497dd37643804fd87f9722c47ea18be")
(setf (get kortex_tools::SaveDataResponse :datatype-) "kortex_tools/SaveDataResponse")
(setf (get kortex_tools::SaveDataResponse :definition-)
      "int64 Save
---
int64 Success

")



(provide :kortex_tools/SaveData "1497dd37643804fd87f9722c47ea18be")


