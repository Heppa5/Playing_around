; Auto-generated. Do not edit!


(cl:in-package mp_mini_picker-srv)


;//! \htmlinclude moveToPointTcp-request.msg.html

(cl:defclass <moveToPointTcp-request> (roslisp-msg-protocol:ros-message)
  ((point
    :reader point
    :initarg :point
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass moveToPointTcp-request (<moveToPointTcp-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <moveToPointTcp-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'moveToPointTcp-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mp_mini_picker-srv:<moveToPointTcp-request> is deprecated: use mp_mini_picker-srv:moveToPointTcp-request instead.")))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <moveToPointTcp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mp_mini_picker-srv:point-val is deprecated.  Use mp_mini_picker-srv:point instead.")
  (point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <moveToPointTcp-request>) ostream)
  "Serializes a message object of type '<moveToPointTcp-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'point))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <moveToPointTcp-request>) istream)
  "Deserializes a message object of type '<moveToPointTcp-request>"
  (cl:setf (cl:slot-value msg 'point) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'point)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<moveToPointTcp-request>)))
  "Returns string type for a service object of type '<moveToPointTcp-request>"
  "mp_mini_picker/moveToPointTcpRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moveToPointTcp-request)))
  "Returns string type for a service object of type 'moveToPointTcp-request"
  "mp_mini_picker/moveToPointTcpRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<moveToPointTcp-request>)))
  "Returns md5sum for a message object of type '<moveToPointTcp-request>"
  "69fd9ff2255d1e9323bb7ce354efb11f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'moveToPointTcp-request)))
  "Returns md5sum for a message object of type 'moveToPointTcp-request"
  "69fd9ff2255d1e9323bb7ce354efb11f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<moveToPointTcp-request>)))
  "Returns full string definition for message of type '<moveToPointTcp-request>"
  (cl:format cl:nil "float64[3] point~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'moveToPointTcp-request)))
  "Returns full string definition for message of type 'moveToPointTcp-request"
  (cl:format cl:nil "float64[3] point~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <moveToPointTcp-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'point) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <moveToPointTcp-request>))
  "Converts a ROS message object to a list"
  (cl:list 'moveToPointTcp-request
    (cl:cons ':point (point msg))
))
;//! \htmlinclude moveToPointTcp-response.msg.html

(cl:defclass <moveToPointTcp-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:fixnum
    :initform 0))
)

(cl:defclass moveToPointTcp-response (<moveToPointTcp-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <moveToPointTcp-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'moveToPointTcp-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mp_mini_picker-srv:<moveToPointTcp-response> is deprecated: use mp_mini_picker-srv:moveToPointTcp-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <moveToPointTcp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mp_mini_picker-srv:ok-val is deprecated.  Use mp_mini_picker-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <moveToPointTcp-response>) ostream)
  "Serializes a message object of type '<moveToPointTcp-response>"
  (cl:let* ((signed (cl:slot-value msg 'ok)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <moveToPointTcp-response>) istream)
  "Deserializes a message object of type '<moveToPointTcp-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ok) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<moveToPointTcp-response>)))
  "Returns string type for a service object of type '<moveToPointTcp-response>"
  "mp_mini_picker/moveToPointTcpResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moveToPointTcp-response)))
  "Returns string type for a service object of type 'moveToPointTcp-response"
  "mp_mini_picker/moveToPointTcpResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<moveToPointTcp-response>)))
  "Returns md5sum for a message object of type '<moveToPointTcp-response>"
  "69fd9ff2255d1e9323bb7ce354efb11f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'moveToPointTcp-response)))
  "Returns md5sum for a message object of type 'moveToPointTcp-response"
  "69fd9ff2255d1e9323bb7ce354efb11f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<moveToPointTcp-response>)))
  "Returns full string definition for message of type '<moveToPointTcp-response>"
  (cl:format cl:nil "int8 ok~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'moveToPointTcp-response)))
  "Returns full string definition for message of type 'moveToPointTcp-response"
  (cl:format cl:nil "int8 ok~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <moveToPointTcp-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <moveToPointTcp-response>))
  "Converts a ROS message object to a list"
  (cl:list 'moveToPointTcp-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'moveToPointTcp)))
  'moveToPointTcp-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'moveToPointTcp)))
  'moveToPointTcp-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moveToPointTcp)))
  "Returns string type for a service object of type '<moveToPointTcp>"
  "mp_mini_picker/moveToPointTcp")