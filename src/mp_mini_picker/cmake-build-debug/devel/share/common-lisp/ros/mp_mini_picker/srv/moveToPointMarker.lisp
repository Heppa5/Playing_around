; Auto-generated. Do not edit!


(cl:in-package mp_mini_picker-srv)


;//! \htmlinclude moveToPointMarker-request.msg.html

(cl:defclass <moveToPointMarker-request> (roslisp-msg-protocol:ros-message)
  ((point
    :reader point
    :initarg :point
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
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

(cl:defclass moveToPointMarker-request (<moveToPointMarker-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <moveToPointMarker-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'moveToPointMarker-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mp_mini_picker-srv:<moveToPointMarker-request> is deprecated: use mp_mini_picker-srv:moveToPointMarker-request instead.")))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <moveToPointMarker-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mp_mini_picker-srv:point-val is deprecated.  Use mp_mini_picker-srv:point instead.")
  (point m))

(cl:ensure-generic-function 'tcpPmarker-val :lambda-list '(m))
(cl:defmethod tcpPmarker-val ((m <moveToPointMarker-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mp_mini_picker-srv:tcpPmarker-val is deprecated.  Use mp_mini_picker-srv:tcpPmarker instead.")
  (tcpPmarker m))

(cl:ensure-generic-function 'tcpRmarker-val :lambda-list '(m))
(cl:defmethod tcpRmarker-val ((m <moveToPointMarker-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mp_mini_picker-srv:tcpRmarker-val is deprecated.  Use mp_mini_picker-srv:tcpRmarker instead.")
  (tcpRmarker m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <moveToPointMarker-request>) ostream)
  "Serializes a message object of type '<moveToPointMarker-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <moveToPointMarker-request>) istream)
  "Deserializes a message object of type '<moveToPointMarker-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<moveToPointMarker-request>)))
  "Returns string type for a service object of type '<moveToPointMarker-request>"
  "mp_mini_picker/moveToPointMarkerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moveToPointMarker-request)))
  "Returns string type for a service object of type 'moveToPointMarker-request"
  "mp_mini_picker/moveToPointMarkerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<moveToPointMarker-request>)))
  "Returns md5sum for a message object of type '<moveToPointMarker-request>"
  "861274133f98039e263a7bec0ea74796")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'moveToPointMarker-request)))
  "Returns md5sum for a message object of type 'moveToPointMarker-request"
  "861274133f98039e263a7bec0ea74796")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<moveToPointMarker-request>)))
  "Returns full string definition for message of type '<moveToPointMarker-request>"
  (cl:format cl:nil "float64[3] point~%float64[3] tcpPmarker~%float64[3] tcpRmarker~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'moveToPointMarker-request)))
  "Returns full string definition for message of type 'moveToPointMarker-request"
  (cl:format cl:nil "float64[3] point~%float64[3] tcpPmarker~%float64[3] tcpRmarker~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <moveToPointMarker-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'point) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'tcpPmarker) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'tcpRmarker) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <moveToPointMarker-request>))
  "Converts a ROS message object to a list"
  (cl:list 'moveToPointMarker-request
    (cl:cons ':point (point msg))
    (cl:cons ':tcpPmarker (tcpPmarker msg))
    (cl:cons ':tcpRmarker (tcpRmarker msg))
))
;//! \htmlinclude moveToPointMarker-response.msg.html

(cl:defclass <moveToPointMarker-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:fixnum
    :initform 0))
)

(cl:defclass moveToPointMarker-response (<moveToPointMarker-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <moveToPointMarker-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'moveToPointMarker-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mp_mini_picker-srv:<moveToPointMarker-response> is deprecated: use mp_mini_picker-srv:moveToPointMarker-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <moveToPointMarker-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mp_mini_picker-srv:ok-val is deprecated.  Use mp_mini_picker-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <moveToPointMarker-response>) ostream)
  "Serializes a message object of type '<moveToPointMarker-response>"
  (cl:let* ((signed (cl:slot-value msg 'ok)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <moveToPointMarker-response>) istream)
  "Deserializes a message object of type '<moveToPointMarker-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ok) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<moveToPointMarker-response>)))
  "Returns string type for a service object of type '<moveToPointMarker-response>"
  "mp_mini_picker/moveToPointMarkerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moveToPointMarker-response)))
  "Returns string type for a service object of type 'moveToPointMarker-response"
  "mp_mini_picker/moveToPointMarkerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<moveToPointMarker-response>)))
  "Returns md5sum for a message object of type '<moveToPointMarker-response>"
  "861274133f98039e263a7bec0ea74796")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'moveToPointMarker-response)))
  "Returns md5sum for a message object of type 'moveToPointMarker-response"
  "861274133f98039e263a7bec0ea74796")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<moveToPointMarker-response>)))
  "Returns full string definition for message of type '<moveToPointMarker-response>"
  (cl:format cl:nil "int8 ok~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'moveToPointMarker-response)))
  "Returns full string definition for message of type 'moveToPointMarker-response"
  (cl:format cl:nil "int8 ok~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <moveToPointMarker-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <moveToPointMarker-response>))
  "Converts a ROS message object to a list"
  (cl:list 'moveToPointMarker-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'moveToPointMarker)))
  'moveToPointMarker-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'moveToPointMarker)))
  'moveToPointMarker-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moveToPointMarker)))
  "Returns string type for a service object of type '<moveToPointMarker>"
  "mp_mini_picker/moveToPointMarker")