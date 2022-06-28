; Auto-generated. Do not edit!


(cl:in-package camera_robot_interaction-srv)


;//! \htmlinclude Movement-request.msg.html

(cl:defclass <Movement-request> (roslisp-msg-protocol:ros-message)
  ((r11
    :reader r11
    :initarg :r11
    :type cl:float
    :initform 0.0)
   (r12
    :reader r12
    :initarg :r12
    :type cl:float
    :initform 0.0)
   (r13
    :reader r13
    :initarg :r13
    :type cl:float
    :initform 0.0)
   (r14
    :reader r14
    :initarg :r14
    :type cl:float
    :initform 0.0)
   (r21
    :reader r21
    :initarg :r21
    :type cl:float
    :initform 0.0)
   (r22
    :reader r22
    :initarg :r22
    :type cl:float
    :initform 0.0)
   (r23
    :reader r23
    :initarg :r23
    :type cl:float
    :initform 0.0)
   (r24
    :reader r24
    :initarg :r24
    :type cl:float
    :initform 0.0)
   (r31
    :reader r31
    :initarg :r31
    :type cl:float
    :initform 0.0)
   (r32
    :reader r32
    :initarg :r32
    :type cl:float
    :initform 0.0)
   (r33
    :reader r33
    :initarg :r33
    :type cl:float
    :initform 0.0)
   (r34
    :reader r34
    :initarg :r34
    :type cl:float
    :initform 0.0)
   (gripper
    :reader gripper
    :initarg :gripper
    :type cl:float
    :initform 0.0)
   (rotate
    :reader rotate
    :initarg :rotate
    :type cl:boolean
    :initform cl:nil)
   (rest_pos
    :reader rest_pos
    :initarg :rest_pos
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Movement-request (<Movement-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Movement-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Movement-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_robot_interaction-srv:<Movement-request> is deprecated: use camera_robot_interaction-srv:Movement-request instead.")))

(cl:ensure-generic-function 'r11-val :lambda-list '(m))
(cl:defmethod r11-val ((m <Movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_robot_interaction-srv:r11-val is deprecated.  Use camera_robot_interaction-srv:r11 instead.")
  (r11 m))

(cl:ensure-generic-function 'r12-val :lambda-list '(m))
(cl:defmethod r12-val ((m <Movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_robot_interaction-srv:r12-val is deprecated.  Use camera_robot_interaction-srv:r12 instead.")
  (r12 m))

(cl:ensure-generic-function 'r13-val :lambda-list '(m))
(cl:defmethod r13-val ((m <Movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_robot_interaction-srv:r13-val is deprecated.  Use camera_robot_interaction-srv:r13 instead.")
  (r13 m))

(cl:ensure-generic-function 'r14-val :lambda-list '(m))
(cl:defmethod r14-val ((m <Movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_robot_interaction-srv:r14-val is deprecated.  Use camera_robot_interaction-srv:r14 instead.")
  (r14 m))

(cl:ensure-generic-function 'r21-val :lambda-list '(m))
(cl:defmethod r21-val ((m <Movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_robot_interaction-srv:r21-val is deprecated.  Use camera_robot_interaction-srv:r21 instead.")
  (r21 m))

(cl:ensure-generic-function 'r22-val :lambda-list '(m))
(cl:defmethod r22-val ((m <Movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_robot_interaction-srv:r22-val is deprecated.  Use camera_robot_interaction-srv:r22 instead.")
  (r22 m))

(cl:ensure-generic-function 'r23-val :lambda-list '(m))
(cl:defmethod r23-val ((m <Movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_robot_interaction-srv:r23-val is deprecated.  Use camera_robot_interaction-srv:r23 instead.")
  (r23 m))

(cl:ensure-generic-function 'r24-val :lambda-list '(m))
(cl:defmethod r24-val ((m <Movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_robot_interaction-srv:r24-val is deprecated.  Use camera_robot_interaction-srv:r24 instead.")
  (r24 m))

(cl:ensure-generic-function 'r31-val :lambda-list '(m))
(cl:defmethod r31-val ((m <Movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_robot_interaction-srv:r31-val is deprecated.  Use camera_robot_interaction-srv:r31 instead.")
  (r31 m))

(cl:ensure-generic-function 'r32-val :lambda-list '(m))
(cl:defmethod r32-val ((m <Movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_robot_interaction-srv:r32-val is deprecated.  Use camera_robot_interaction-srv:r32 instead.")
  (r32 m))

(cl:ensure-generic-function 'r33-val :lambda-list '(m))
(cl:defmethod r33-val ((m <Movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_robot_interaction-srv:r33-val is deprecated.  Use camera_robot_interaction-srv:r33 instead.")
  (r33 m))

(cl:ensure-generic-function 'r34-val :lambda-list '(m))
(cl:defmethod r34-val ((m <Movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_robot_interaction-srv:r34-val is deprecated.  Use camera_robot_interaction-srv:r34 instead.")
  (r34 m))

(cl:ensure-generic-function 'gripper-val :lambda-list '(m))
(cl:defmethod gripper-val ((m <Movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_robot_interaction-srv:gripper-val is deprecated.  Use camera_robot_interaction-srv:gripper instead.")
  (gripper m))

(cl:ensure-generic-function 'rotate-val :lambda-list '(m))
(cl:defmethod rotate-val ((m <Movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_robot_interaction-srv:rotate-val is deprecated.  Use camera_robot_interaction-srv:rotate instead.")
  (rotate m))

(cl:ensure-generic-function 'rest_pos-val :lambda-list '(m))
(cl:defmethod rest_pos-val ((m <Movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_robot_interaction-srv:rest_pos-val is deprecated.  Use camera_robot_interaction-srv:rest_pos instead.")
  (rest_pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Movement-request>) ostream)
  "Serializes a message object of type '<Movement-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'r11))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'r12))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'r13))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'r14))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'r21))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'r22))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'r23))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'r24))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'r31))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'r32))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'r33))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'r34))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gripper))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'rotate) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'rest_pos) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Movement-request>) istream)
  "Deserializes a message object of type '<Movement-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r11) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r12) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r13) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r14) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r21) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r22) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r23) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r24) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r31) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r32) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r33) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r34) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gripper) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'rotate) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'rest_pos) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Movement-request>)))
  "Returns string type for a service object of type '<Movement-request>"
  "camera_robot_interaction/MovementRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Movement-request)))
  "Returns string type for a service object of type 'Movement-request"
  "camera_robot_interaction/MovementRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Movement-request>)))
  "Returns md5sum for a message object of type '<Movement-request>"
  "4787e27e3721f1f2004cf17997b3e3ae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Movement-request)))
  "Returns md5sum for a message object of type 'Movement-request"
  "4787e27e3721f1f2004cf17997b3e3ae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Movement-request>)))
  "Returns full string definition for message of type '<Movement-request>"
  (cl:format cl:nil "float32 r11~%float32 r12~%float32 r13~%float32 r14~%float32 r21~%float32 r22~%float32 r23~%float32 r24~%float32 r31~%float32 r32~%float32 r33~%float32 r34~%float32 gripper~%bool rotate~%bool rest_pos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Movement-request)))
  "Returns full string definition for message of type 'Movement-request"
  (cl:format cl:nil "float32 r11~%float32 r12~%float32 r13~%float32 r14~%float32 r21~%float32 r22~%float32 r23~%float32 r24~%float32 r31~%float32 r32~%float32 r33~%float32 r34~%float32 gripper~%bool rotate~%bool rest_pos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Movement-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Movement-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Movement-request
    (cl:cons ':r11 (r11 msg))
    (cl:cons ':r12 (r12 msg))
    (cl:cons ':r13 (r13 msg))
    (cl:cons ':r14 (r14 msg))
    (cl:cons ':r21 (r21 msg))
    (cl:cons ':r22 (r22 msg))
    (cl:cons ':r23 (r23 msg))
    (cl:cons ':r24 (r24 msg))
    (cl:cons ':r31 (r31 msg))
    (cl:cons ':r32 (r32 msg))
    (cl:cons ':r33 (r33 msg))
    (cl:cons ':r34 (r34 msg))
    (cl:cons ':gripper (gripper msg))
    (cl:cons ':rotate (rotate msg))
    (cl:cons ':rest_pos (rest_pos msg))
))
;//! \htmlinclude Movement-response.msg.html

(cl:defclass <Movement-response> (roslisp-msg-protocol:ros-message)
  ((Success
    :reader Success
    :initarg :Success
    :type cl:integer
    :initform 0)
   (gripper_fb
    :reader gripper_fb
    :initarg :gripper_fb
    :type cl:float
    :initform 0.0))
)

(cl:defclass Movement-response (<Movement-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Movement-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Movement-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_robot_interaction-srv:<Movement-response> is deprecated: use camera_robot_interaction-srv:Movement-response instead.")))

(cl:ensure-generic-function 'Success-val :lambda-list '(m))
(cl:defmethod Success-val ((m <Movement-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_robot_interaction-srv:Success-val is deprecated.  Use camera_robot_interaction-srv:Success instead.")
  (Success m))

(cl:ensure-generic-function 'gripper_fb-val :lambda-list '(m))
(cl:defmethod gripper_fb-val ((m <Movement-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_robot_interaction-srv:gripper_fb-val is deprecated.  Use camera_robot_interaction-srv:gripper_fb instead.")
  (gripper_fb m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Movement-response>) ostream)
  "Serializes a message object of type '<Movement-response>"
  (cl:let* ((signed (cl:slot-value msg 'Success)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gripper_fb))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Movement-response>) istream)
  "Deserializes a message object of type '<Movement-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Success) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gripper_fb) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Movement-response>)))
  "Returns string type for a service object of type '<Movement-response>"
  "camera_robot_interaction/MovementResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Movement-response)))
  "Returns string type for a service object of type 'Movement-response"
  "camera_robot_interaction/MovementResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Movement-response>)))
  "Returns md5sum for a message object of type '<Movement-response>"
  "4787e27e3721f1f2004cf17997b3e3ae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Movement-response)))
  "Returns md5sum for a message object of type 'Movement-response"
  "4787e27e3721f1f2004cf17997b3e3ae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Movement-response>)))
  "Returns full string definition for message of type '<Movement-response>"
  (cl:format cl:nil "int64 Success~%float32 gripper_fb~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Movement-response)))
  "Returns full string definition for message of type 'Movement-response"
  (cl:format cl:nil "int64 Success~%float32 gripper_fb~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Movement-response>))
  (cl:+ 0
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Movement-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Movement-response
    (cl:cons ':Success (Success msg))
    (cl:cons ':gripper_fb (gripper_fb msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Movement)))
  'Movement-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Movement)))
  'Movement-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Movement)))
  "Returns string type for a service object of type '<Movement>"
  "camera_robot_interaction/Movement")