; Auto-generated. Do not edit!


(cl:in-package kortex_movement-srv)


;//! \htmlinclude cartesian_movement-request.msg.html

(cl:defclass <cartesian_movement-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (thetax
    :reader thetax
    :initarg :thetax
    :type cl:float
    :initform 0.0)
   (thetay
    :reader thetay
    :initarg :thetay
    :type cl:float
    :initform 0.0)
   (thetaz
    :reader thetaz
    :initarg :thetaz
    :type cl:float
    :initform 0.0))
)

(cl:defclass cartesian_movement-request (<cartesian_movement-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cartesian_movement-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cartesian_movement-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_movement-srv:<cartesian_movement-request> is deprecated: use kortex_movement-srv:cartesian_movement-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <cartesian_movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_movement-srv:x-val is deprecated.  Use kortex_movement-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <cartesian_movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_movement-srv:y-val is deprecated.  Use kortex_movement-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <cartesian_movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_movement-srv:z-val is deprecated.  Use kortex_movement-srv:z instead.")
  (z m))

(cl:ensure-generic-function 'thetax-val :lambda-list '(m))
(cl:defmethod thetax-val ((m <cartesian_movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_movement-srv:thetax-val is deprecated.  Use kortex_movement-srv:thetax instead.")
  (thetax m))

(cl:ensure-generic-function 'thetay-val :lambda-list '(m))
(cl:defmethod thetay-val ((m <cartesian_movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_movement-srv:thetay-val is deprecated.  Use kortex_movement-srv:thetay instead.")
  (thetay m))

(cl:ensure-generic-function 'thetaz-val :lambda-list '(m))
(cl:defmethod thetaz-val ((m <cartesian_movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_movement-srv:thetaz-val is deprecated.  Use kortex_movement-srv:thetaz instead.")
  (thetaz m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cartesian_movement-request>) ostream)
  "Serializes a message object of type '<cartesian_movement-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thetax))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thetay))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thetaz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cartesian_movement-request>) istream)
  "Deserializes a message object of type '<cartesian_movement-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thetax) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thetay) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thetaz) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cartesian_movement-request>)))
  "Returns string type for a service object of type '<cartesian_movement-request>"
  "kortex_movement/cartesian_movementRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cartesian_movement-request)))
  "Returns string type for a service object of type 'cartesian_movement-request"
  "kortex_movement/cartesian_movementRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cartesian_movement-request>)))
  "Returns md5sum for a message object of type '<cartesian_movement-request>"
  "527d545043520e6207cc8929405c4e9e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cartesian_movement-request)))
  "Returns md5sum for a message object of type 'cartesian_movement-request"
  "527d545043520e6207cc8929405c4e9e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cartesian_movement-request>)))
  "Returns full string definition for message of type '<cartesian_movement-request>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%float32 thetax~%float32 thetay~%float32 thetaz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cartesian_movement-request)))
  "Returns full string definition for message of type 'cartesian_movement-request"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%float32 thetax~%float32 thetay~%float32 thetaz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cartesian_movement-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cartesian_movement-request>))
  "Converts a ROS message object to a list"
  (cl:list 'cartesian_movement-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':thetax (thetax msg))
    (cl:cons ':thetay (thetay msg))
    (cl:cons ':thetaz (thetaz msg))
))
;//! \htmlinclude cartesian_movement-response.msg.html

(cl:defclass <cartesian_movement-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass cartesian_movement-response (<cartesian_movement-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cartesian_movement-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cartesian_movement-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_movement-srv:<cartesian_movement-response> is deprecated: use kortex_movement-srv:cartesian_movement-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <cartesian_movement-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_movement-srv:output-val is deprecated.  Use kortex_movement-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cartesian_movement-response>) ostream)
  "Serializes a message object of type '<cartesian_movement-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'output) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cartesian_movement-response>) istream)
  "Deserializes a message object of type '<cartesian_movement-response>"
    (cl:setf (cl:slot-value msg 'output) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cartesian_movement-response>)))
  "Returns string type for a service object of type '<cartesian_movement-response>"
  "kortex_movement/cartesian_movementResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cartesian_movement-response)))
  "Returns string type for a service object of type 'cartesian_movement-response"
  "kortex_movement/cartesian_movementResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cartesian_movement-response>)))
  "Returns md5sum for a message object of type '<cartesian_movement-response>"
  "527d545043520e6207cc8929405c4e9e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cartesian_movement-response)))
  "Returns md5sum for a message object of type 'cartesian_movement-response"
  "527d545043520e6207cc8929405c4e9e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cartesian_movement-response>)))
  "Returns full string definition for message of type '<cartesian_movement-response>"
  (cl:format cl:nil "bool output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cartesian_movement-response)))
  "Returns full string definition for message of type 'cartesian_movement-response"
  (cl:format cl:nil "bool output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cartesian_movement-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cartesian_movement-response>))
  "Converts a ROS message object to a list"
  (cl:list 'cartesian_movement-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'cartesian_movement)))
  'cartesian_movement-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'cartesian_movement)))
  'cartesian_movement-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cartesian_movement)))
  "Returns string type for a service object of type '<cartesian_movement>"
  "kortex_movement/cartesian_movement")