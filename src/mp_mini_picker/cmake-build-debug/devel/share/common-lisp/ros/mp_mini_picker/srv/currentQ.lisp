; Auto-generated. Do not edit!


(cl:in-package mp_mini_picker-srv)


;//! \htmlinclude currentQ-request.msg.html

(cl:defclass <currentQ-request> (roslisp-msg-protocol:ros-message)
  ((getQ
    :reader getQ
    :initarg :getQ
    :type cl:fixnum
    :initform 0))
)

(cl:defclass currentQ-request (<currentQ-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <currentQ-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'currentQ-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mp_mini_picker-srv:<currentQ-request> is deprecated: use mp_mini_picker-srv:currentQ-request instead.")))

(cl:ensure-generic-function 'getQ-val :lambda-list '(m))
(cl:defmethod getQ-val ((m <currentQ-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mp_mini_picker-srv:getQ-val is deprecated.  Use mp_mini_picker-srv:getQ instead.")
  (getQ m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <currentQ-request>) ostream)
  "Serializes a message object of type '<currentQ-request>"
  (cl:let* ((signed (cl:slot-value msg 'getQ)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <currentQ-request>) istream)
  "Deserializes a message object of type '<currentQ-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'getQ) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<currentQ-request>)))
  "Returns string type for a service object of type '<currentQ-request>"
  "mp_mini_picker/currentQRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'currentQ-request)))
  "Returns string type for a service object of type 'currentQ-request"
  "mp_mini_picker/currentQRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<currentQ-request>)))
  "Returns md5sum for a message object of type '<currentQ-request>"
  "0abad29c7af93c5a38bad143c421d80d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'currentQ-request)))
  "Returns md5sum for a message object of type 'currentQ-request"
  "0abad29c7af93c5a38bad143c421d80d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<currentQ-request>)))
  "Returns full string definition for message of type '<currentQ-request>"
  (cl:format cl:nil "int8 getQ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'currentQ-request)))
  "Returns full string definition for message of type 'currentQ-request"
  (cl:format cl:nil "int8 getQ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <currentQ-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <currentQ-request>))
  "Converts a ROS message object to a list"
  (cl:list 'currentQ-request
    (cl:cons ':getQ (getQ msg))
))
;//! \htmlinclude currentQ-response.msg.html

(cl:defclass <currentQ-response> (roslisp-msg-protocol:ros-message)
  ((Q
    :reader Q
    :initarg :Q
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass currentQ-response (<currentQ-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <currentQ-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'currentQ-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mp_mini_picker-srv:<currentQ-response> is deprecated: use mp_mini_picker-srv:currentQ-response instead.")))

(cl:ensure-generic-function 'Q-val :lambda-list '(m))
(cl:defmethod Q-val ((m <currentQ-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mp_mini_picker-srv:Q-val is deprecated.  Use mp_mini_picker-srv:Q instead.")
  (Q m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <currentQ-response>) ostream)
  "Serializes a message object of type '<currentQ-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <currentQ-response>) istream)
  "Deserializes a message object of type '<currentQ-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<currentQ-response>)))
  "Returns string type for a service object of type '<currentQ-response>"
  "mp_mini_picker/currentQResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'currentQ-response)))
  "Returns string type for a service object of type 'currentQ-response"
  "mp_mini_picker/currentQResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<currentQ-response>)))
  "Returns md5sum for a message object of type '<currentQ-response>"
  "0abad29c7af93c5a38bad143c421d80d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'currentQ-response)))
  "Returns md5sum for a message object of type 'currentQ-response"
  "0abad29c7af93c5a38bad143c421d80d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<currentQ-response>)))
  "Returns full string definition for message of type '<currentQ-response>"
  (cl:format cl:nil "float64[6] Q~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'currentQ-response)))
  "Returns full string definition for message of type 'currentQ-response"
  (cl:format cl:nil "float64[6] Q~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <currentQ-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'Q) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <currentQ-response>))
  "Converts a ROS message object to a list"
  (cl:list 'currentQ-response
    (cl:cons ':Q (Q msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'currentQ)))
  'currentQ-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'currentQ)))
  'currentQ-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'currentQ)))
  "Returns string type for a service object of type '<currentQ>"
  "mp_mini_picker/currentQ")