; Auto-generated. Do not edit!


(cl:in-package mp_mini_picker-srv)


;//! \htmlinclude moveToPoseMarker-request.msg.html

(cl:defclass <moveToPoseMarker-request> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (tcpPmarker
    :reader tcpPmarker
    :initarg :tcpPmarker
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (tcpRmarker
    :reader tcpRmarker
    :initarg :tcpRmarker
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass moveToPoseMarker-request (<moveToPoseMarker-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <moveToPoseMarker-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'moveToPoseMarker-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mp_mini_picker-srv:<moveToPoseMarker-request> is deprecated: use mp_mini_picker-srv:moveToPoseMarker-request instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <moveToPoseMarker-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mp_mini_picker-srv:pose-val is deprecated.  Use mp_mini_picker-srv:pose instead.")
  (pose m))

(cl:ensure-generic-function 'tcpPmarker-val :lambda-list '(m))
(cl:defmethod tcpPmarker-val ((m <moveToPoseMarker-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mp_mini_picker-srv:tcpPmarker-val is deprecated.  Use mp_mini_picker-srv:tcpPmarker instead.")
  (tcpPmarker m))

(cl:ensure-generic-function 'tcpRmarker-val :lambda-list '(m))
(cl:defmethod tcpRmarker-val ((m <moveToPoseMarker-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mp_mini_picker-srv:tcpRmarker-val is deprecated.  Use mp_mini_picker-srv:tcpRmarker instead.")
  (tcpRmarker m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <moveToPoseMarker-request>) ostream)
  "Serializes a message object of type '<moveToPoseMarker-request>"
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
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'tcpPmarker))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'tcpRmarker))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <moveToPoseMarker-request>) istream)
  "Deserializes a message object of type '<moveToPoseMarker-request>"
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
  (cl:setf (cl:slot-value msg 'tcpPmarker) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'tcpPmarker)))
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
  (cl:setf (cl:slot-value msg 'tcpRmarker) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'tcpRmarker)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<moveToPoseMarker-request>)))
  "Returns string type for a service object of type '<moveToPoseMarker-request>"
  "mp_mini_picker/moveToPoseMarkerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moveToPoseMarker-request)))
  "Returns string type for a service object of type 'moveToPoseMarker-request"
  "mp_mini_picker/moveToPoseMarkerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<moveToPoseMarker-request>)))
  "Returns md5sum for a message object of type '<moveToPoseMarker-request>"
  "90ff3d42ea6ed49171a1cd36008690f4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'moveToPoseMarker-request)))
  "Returns md5sum for a message object of type 'moveToPoseMarker-request"
  "90ff3d42ea6ed49171a1cd36008690f4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<moveToPoseMarker-request>)))
  "Returns full string definition for message of type '<moveToPoseMarker-request>"
  (cl:format cl:nil "float64[6] pose~%float64[3] tcpPmarker~%float64[3] tcpRmarker~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'moveToPoseMarker-request)))
  "Returns full string definition for message of type 'moveToPoseMarker-request"
  (cl:format cl:nil "float64[6] pose~%float64[3] tcpPmarker~%float64[3] tcpRmarker~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <moveToPoseMarker-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pose) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'tcpPmarker) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'tcpRmarker) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <moveToPoseMarker-request>))
  "Converts a ROS message object to a list"
  (cl:list 'moveToPoseMarker-request
    (cl:cons ':pose (pose msg))
    (cl:cons ':tcpPmarker (tcpPmarker msg))
    (cl:cons ':tcpRmarker (tcpRmarker msg))
))
;//! \htmlinclude moveToPoseMarker-response.msg.html

(cl:defclass <moveToPoseMarker-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:fixnum
    :initform 0))
)

(cl:defclass moveToPoseMarker-response (<moveToPoseMarker-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <moveToPoseMarker-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'moveToPoseMarker-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mp_mini_picker-srv:<moveToPoseMarker-response> is deprecated: use mp_mini_picker-srv:moveToPoseMarker-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <moveToPoseMarker-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mp_mini_picker-srv:ok-val is deprecated.  Use mp_mini_picker-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <moveToPoseMarker-response>) ostream)
  "Serializes a message object of type '<moveToPoseMarker-response>"
  (cl:let* ((signed (cl:slot-value msg 'ok)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <moveToPoseMarker-response>) istream)
  "Deserializes a message object of type '<moveToPoseMarker-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ok) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<moveToPoseMarker-response>)))
  "Returns string type for a service object of type '<moveToPoseMarker-response>"
  "mp_mini_picker/moveToPoseMarkerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moveToPoseMarker-response)))
  "Returns string type for a service object of type 'moveToPoseMarker-response"
  "mp_mini_picker/moveToPoseMarkerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<moveToPoseMarker-response>)))
  "Returns md5sum for a message object of type '<moveToPoseMarker-response>"
  "90ff3d42ea6ed49171a1cd36008690f4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'moveToPoseMarker-response)))
  "Returns md5sum for a message object of type 'moveToPoseMarker-response"
  "90ff3d42ea6ed49171a1cd36008690f4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<moveToPoseMarker-response>)))
  "Returns full string definition for message of type '<moveToPoseMarker-response>"
  (cl:format cl:nil "int8 ok~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'moveToPoseMarker-response)))
  "Returns full string definition for message of type 'moveToPoseMarker-response"
  (cl:format cl:nil "int8 ok~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <moveToPoseMarker-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <moveToPoseMarker-response>))
  "Converts a ROS message object to a list"
  (cl:list 'moveToPoseMarker-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'moveToPoseMarker)))
  'moveToPoseMarker-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'moveToPoseMarker)))
  'moveToPoseMarker-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moveToPoseMarker)))
  "Returns string type for a service object of type '<moveToPoseMarker>"
  "mp_mini_picker/moveToPoseMarker")