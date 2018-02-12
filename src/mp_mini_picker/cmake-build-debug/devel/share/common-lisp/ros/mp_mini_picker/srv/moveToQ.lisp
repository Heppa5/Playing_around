; Auto-generated. Do not edit!


(cl:in-package mp_mini_picker-srv)


;//! \htmlinclude moveToQ-request.msg.html

(cl:defclass <moveToQ-request> (roslisp-msg-protocol:ros-message)
  ((Q
    :reader Q
    :initarg :Q
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass moveToQ-request (<moveToQ-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <moveToQ-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'moveToQ-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mp_mini_picker-srv:<moveToQ-request> is deprecated: use mp_mini_picker-srv:moveToQ-request instead.")))

(cl:ensure-generic-function 'Q-val :lambda-list '(m))
(cl:defmethod Q-val ((m <moveToQ-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mp_mini_picker-srv:Q-val is deprecated.  Use mp_mini_picker-srv:Q instead.")
  (Q m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <moveToQ-request>) ostream)
  "Serializes a message object of type '<moveToQ-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'Q))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <moveToQ-request>) istream)
  "Deserializes a message object of type '<moveToQ-request>"
  (cl:setf (cl:slot-value msg 'Q) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'Q)))
    (cl:dotimes (i 6)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<moveToQ-request>)))
  "Returns string type for a service object of type '<moveToQ-request>"
  "mp_mini_picker/moveToQRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moveToQ-request)))
  "Returns string type for a service object of type 'moveToQ-request"
  "mp_mini_picker/moveToQRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<moveToQ-request>)))
  "Returns md5sum for a message object of type '<moveToQ-request>"
  "1d1efbf9948db38dda60ea143a471260")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'moveToQ-request)))
  "Returns md5sum for a message object of type 'moveToQ-request"
  "1d1efbf9948db38dda60ea143a471260")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<moveToQ-request>)))
  "Returns full string definition for message of type '<moveToQ-request>"
  (cl:format cl:nil "float64[6] Q~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'moveToQ-request)))
  "Returns full string definition for message of type 'moveToQ-request"
  (cl:format cl:nil "float64[6] Q~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <moveToQ-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'Q) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <moveToQ-request>))
  "Converts a ROS message object to a list"
  (cl:list 'moveToQ-request
    (cl:cons ':Q (Q msg))
))
;//! \htmlinclude moveToQ-response.msg.html

(cl:defclass <moveToQ-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:fixnum
    :initform 0))
)

(cl:defclass moveToQ-response (<moveToQ-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <moveToQ-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'moveToQ-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mp_mini_picker-srv:<moveToQ-response> is deprecated: use mp_mini_picker-srv:moveToQ-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <moveToQ-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mp_mini_picker-srv:ok-val is deprecated.  Use mp_mini_picker-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <moveToQ-response>) ostream)
  "Serializes a message object of type '<moveToQ-response>"
  (cl:let* ((signed (cl:slot-value msg 'ok)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <moveToQ-response>) istream)
  "Deserializes a message object of type '<moveToQ-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ok) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<moveToQ-response>)))
  "Returns string type for a service object of type '<moveToQ-response>"
  "mp_mini_picker/moveToQResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moveToQ-response)))
  "Returns string type for a service object of type 'moveToQ-response"
  "mp_mini_picker/moveToQResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<moveToQ-response>)))
  "Returns md5sum for a message object of type '<moveToQ-response>"
  "1d1efbf9948db38dda60ea143a471260")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'moveToQ-response)))
  "Returns md5sum for a message object of type 'moveToQ-response"
  "1d1efbf9948db38dda60ea143a471260")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<moveToQ-response>)))
  "Returns full string definition for message of type '<moveToQ-response>"
  (cl:format cl:nil "int8 ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'moveToQ-response)))
  "Returns full string definition for message of type 'moveToQ-response"
  (cl:format cl:nil "int8 ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <moveToQ-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <moveToQ-response>))
  "Converts a ROS message object to a list"
  (cl:list 'moveToQ-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'moveToQ)))
  'moveToQ-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'moveToQ)))
  'moveToQ-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moveToQ)))
  "Returns string type for a service object of type '<moveToQ>"
  "mp_mini_picker/moveToQ")