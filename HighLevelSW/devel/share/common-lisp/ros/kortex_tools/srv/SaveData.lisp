; Auto-generated. Do not edit!


(cl:in-package kortex_tools-srv)


;//! \htmlinclude SaveData-request.msg.html

(cl:defclass <SaveData-request> (roslisp-msg-protocol:ros-message)
  ((Save
    :reader Save
    :initarg :Save
    :type cl:integer
    :initform 0))
)

(cl:defclass SaveData-request (<SaveData-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SaveData-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SaveData-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_tools-srv:<SaveData-request> is deprecated: use kortex_tools-srv:SaveData-request instead.")))

(cl:ensure-generic-function 'Save-val :lambda-list '(m))
(cl:defmethod Save-val ((m <SaveData-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_tools-srv:Save-val is deprecated.  Use kortex_tools-srv:Save instead.")
  (Save m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SaveData-request>) ostream)
  "Serializes a message object of type '<SaveData-request>"
  (cl:let* ((signed (cl:slot-value msg 'Save)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SaveData-request>) istream)
  "Deserializes a message object of type '<SaveData-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Save) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SaveData-request>)))
  "Returns string type for a service object of type '<SaveData-request>"
  "kortex_tools/SaveDataRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SaveData-request)))
  "Returns string type for a service object of type 'SaveData-request"
  "kortex_tools/SaveDataRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SaveData-request>)))
  "Returns md5sum for a message object of type '<SaveData-request>"
  "1497dd37643804fd87f9722c47ea18be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SaveData-request)))
  "Returns md5sum for a message object of type 'SaveData-request"
  "1497dd37643804fd87f9722c47ea18be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SaveData-request>)))
  "Returns full string definition for message of type '<SaveData-request>"
  (cl:format cl:nil "int64 Save~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SaveData-request)))
  "Returns full string definition for message of type 'SaveData-request"
  (cl:format cl:nil "int64 Save~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SaveData-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SaveData-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SaveData-request
    (cl:cons ':Save (Save msg))
))
;//! \htmlinclude SaveData-response.msg.html

(cl:defclass <SaveData-response> (roslisp-msg-protocol:ros-message)
  ((Success
    :reader Success
    :initarg :Success
    :type cl:integer
    :initform 0))
)

(cl:defclass SaveData-response (<SaveData-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SaveData-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SaveData-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_tools-srv:<SaveData-response> is deprecated: use kortex_tools-srv:SaveData-response instead.")))

(cl:ensure-generic-function 'Success-val :lambda-list '(m))
(cl:defmethod Success-val ((m <SaveData-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_tools-srv:Success-val is deprecated.  Use kortex_tools-srv:Success instead.")
  (Success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SaveData-response>) ostream)
  "Serializes a message object of type '<SaveData-response>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SaveData-response>) istream)
  "Deserializes a message object of type '<SaveData-response>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SaveData-response>)))
  "Returns string type for a service object of type '<SaveData-response>"
  "kortex_tools/SaveDataResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SaveData-response)))
  "Returns string type for a service object of type 'SaveData-response"
  "kortex_tools/SaveDataResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SaveData-response>)))
  "Returns md5sum for a message object of type '<SaveData-response>"
  "1497dd37643804fd87f9722c47ea18be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SaveData-response)))
  "Returns md5sum for a message object of type 'SaveData-response"
  "1497dd37643804fd87f9722c47ea18be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SaveData-response>)))
  "Returns full string definition for message of type '<SaveData-response>"
  (cl:format cl:nil "int64 Success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SaveData-response)))
  "Returns full string definition for message of type 'SaveData-response"
  (cl:format cl:nil "int64 Success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SaveData-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SaveData-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SaveData-response
    (cl:cons ':Success (Success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SaveData)))
  'SaveData-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SaveData)))
  'SaveData-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SaveData)))
  "Returns string type for a service object of type '<SaveData>"
  "kortex_tools/SaveData")