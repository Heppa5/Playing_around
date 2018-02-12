; Auto-generated. Do not edit!


(cl:in-package mp_mini_picker-srv)


;//! \htmlinclude moveToPoseTcp-request.msg.html

(cl:defclass <moveToPoseTcp-request> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass moveToPoseTcp-request (<moveToPoseTcp-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <moveToPoseTcp-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'moveToPoseTcp-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mp_mini_picker-srv:<moveToPoseTcp-request> is deprecated: use mp_mini_picker-srv:moveToPoseTcp-request instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <moveToPoseTcp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mp_mini_picker-srv:pose-val is deprecated.  Use mp_mini_picker-srv:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <moveToPoseTcp-request>) ostream)
  "Serializes a message object of type '<moveToPoseTcp-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'pose))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <moveToPoseTcp-request>) istream)
  "Deserializes a message object of type '<moveToPoseTcp-request>"
  (cl:setf (cl:slot-value msg 'pose) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'pose)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<moveToPoseTcp-request>)))
  "Returns string type for a service object of type '<moveToPoseTcp-request>"
  "mp_mini_picker/moveToPoseTcpRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moveToPoseTcp-request)))
  "Returns string type for a service object of type 'moveToPoseTcp-request"
  "mp_mini_picker/moveToPoseTcpRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<moveToPoseTcp-request>)))
  "Returns md5sum for a message object of type '<moveToPoseTcp-request>"
  "c25f95a4c81298b09162de739af2fd0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'moveToPoseTcp-request)))
  "Returns md5sum for a message object of type 'moveToPoseTcp-request"
  "c25f95a4c81298b09162de739af2fd0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<moveToPoseTcp-request>)))
  "Returns full string definition for message of type '<moveToPoseTcp-request>"
  (cl:format cl:nil "float64[6] pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'moveToPoseTcp-request)))
  "Returns full string definition for message of type 'moveToPoseTcp-request"
  (cl:format cl:nil "float64[6] pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <moveToPoseTcp-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pose) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <moveToPoseTcp-request>))
  "Converts a ROS message object to a list"
  (cl:list 'moveToPoseTcp-request
    (cl:cons ':pose (pose msg))
))
;//! \htmlinclude moveToPoseTcp-response.msg.html

(cl:defclass <moveToPoseTcp-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:fixnum
    :initform 0))
)

(cl:defclass moveToPoseTcp-response (<moveToPoseTcp-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <moveToPoseTcp-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'moveToPoseTcp-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mp_mini_picker-srv:<moveToPoseTcp-response> is deprecated: use mp_mini_picker-srv:moveToPoseTcp-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <moveToPoseTcp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mp_mini_picker-srv:ok-val is deprecated.  Use mp_mini_picker-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <moveToPoseTcp-response>) ostream)
  "Serializes a message object of type '<moveToPoseTcp-response>"
  (cl:let* ((signed (cl:slot-value msg 'ok)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <moveToPoseTcp-response>) istream)
  "Deserializes a message object of type '<moveToPoseTcp-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ok) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<moveToPoseTcp-response>)))
  "Returns string type for a service object of type '<moveToPoseTcp-response>"
  "mp_mini_picker/moveToPoseTcpResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moveToPoseTcp-response)))
  "Returns string type for a service object of type 'moveToPoseTcp-response"
  "mp_mini_picker/moveToPoseTcpResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<moveToPoseTcp-response>)))
  "Returns md5sum for a message object of type '<moveToPoseTcp-response>"
  "c25f95a4c81298b09162de739af2fd0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'moveToPoseTcp-response)))
  "Returns md5sum for a message object of type 'moveToPoseTcp-response"
  "c25f95a4c81298b09162de739af2fd0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<moveToPoseTcp-response>)))
  "Returns full string definition for message of type '<moveToPoseTcp-response>"
  (cl:format cl:nil "int8 ok~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'moveToPoseTcp-response)))
  "Returns full string definition for message of type 'moveToPoseTcp-response"
  (cl:format cl:nil "int8 ok~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <moveToPoseTcp-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <moveToPoseTcp-response>))
  "Converts a ROS message object to a list"
  (cl:list 'moveToPoseTcp-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'moveToPoseTcp)))
  'moveToPoseTcp-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'moveToPoseTcp)))
  'moveToPoseTcp-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moveToPoseTcp)))
  "Returns string type for a service object of type '<moveToPoseTcp>"
  "mp_mini_picker/moveToPoseTcp")