;; Auto-generated. Do not edit!


(when (boundp 'mp_mini_picker::moveToPointMarker)
  (if (not (find-package "MP_MINI_PICKER"))
    (make-package "MP_MINI_PICKER"))
  (shadow 'moveToPointMarker (find-package "MP_MINI_PICKER")))
(unless (find-package "MP_MINI_PICKER::MOVETOPOINTMARKER")
  (make-package "MP_MINI_PICKER::MOVETOPOINTMARKER"))
(unless (find-package "MP_MINI_PICKER::MOVETOPOINTMARKERREQUEST")
  (make-package "MP_MINI_PICKER::MOVETOPOINTMARKERREQUEST"))
(unless (find-package "MP_MINI_PICKER::MOVETOPOINTMARKERRESPONSE")
  (make-package "MP_MINI_PICKER::MOVETOPOINTMARKERRESPONSE"))

(in-package "ROS")





(defclass mp_mini_picker::moveToPointMarkerRequest
  :super ros::object
  :slots (_point _tcpPmarker _tcpRmarker ))

(defmethod mp_mini_picker::moveToPointMarkerRequest
  (:init
   (&key
    ((:point __point) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:tcpPmarker __tcpPmarker) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:tcpRmarker __tcpRmarker) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _point __point)
   (setq _tcpPmarker __tcpPmarker)
   (setq _tcpRmarker __tcpRmarker)
   self)
  (:point
   (&optional __point)
   (if __point (setq _point __point)) _point)
  (:tcpPmarker
   (&optional __tcpPmarker)
   (if __tcpPmarker (setq _tcpPmarker __tcpPmarker)) _tcpPmarker)
  (:tcpRmarker
   (&optional __tcpRmarker)
   (if __tcpRmarker (setq _tcpRmarker __tcpRmarker)) _tcpRmarker)
  (:serialization-length
   ()
   (+
    ;; float64[3] _point
    (* 8    3)
    ;; float64[3] _tcpPmarker
    (* 8    3)
    ;; float64[3] _tcpRmarker
    (* 8    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[3] _point
     (dotimes (i 3)
       (sys::poke (elt _point i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _tcpPmarker
     (dotimes (i 3)
       (sys::poke (elt _tcpPmarker i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _tcpRmarker
     (dotimes (i 3)
       (sys::poke (elt _tcpRmarker i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[3] _point
   (dotimes (i (length _point))
     (setf (elt _point i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _tcpPmarker
   (dotimes (i (length _tcpPmarker))
     (setf (elt _tcpPmarker i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _tcpRmarker
   (dotimes (i (length _tcpRmarker))
     (setf (elt _tcpRmarker i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass mp_mini_picker::moveToPointMarkerResponse
  :super ros::object
  :slots (_ok ))

(defmethod mp_mini_picker::moveToPointMarkerResponse
  (:init
   (&key
    ((:ok __ok) 0)
    )
   (send-super :init)
   (setq _ok (round __ok))
   self)
  (:ok
   (&optional __ok)
   (if __ok (setq _ok __ok)) _ok)
  (:serialization-length
   ()
   (+
    ;; int8 _ok
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _ok
       (write-byte _ok s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _ok
     (setq _ok (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ok 127) (setq _ok (- _ok 256)))
   ;;
   self)
  )

(defclass mp_mini_picker::moveToPointMarker
  :super ros::object
  :slots ())

(setf (get mp_mini_picker::moveToPointMarker :md5sum-) "861274133f98039e263a7bec0ea74796")
(setf (get mp_mini_picker::moveToPointMarker :datatype-) "mp_mini_picker/moveToPointMarker")
(setf (get mp_mini_picker::moveToPointMarker :request) mp_mini_picker::moveToPointMarkerRequest)
(setf (get mp_mini_picker::moveToPointMarker :response) mp_mini_picker::moveToPointMarkerResponse)

(defmethod mp_mini_picker::moveToPointMarkerRequest
  (:response () (instance mp_mini_picker::moveToPointMarkerResponse :init)))

(setf (get mp_mini_picker::moveToPointMarkerRequest :md5sum-) "861274133f98039e263a7bec0ea74796")
(setf (get mp_mini_picker::moveToPointMarkerRequest :datatype-) "mp_mini_picker/moveToPointMarkerRequest")
(setf (get mp_mini_picker::moveToPointMarkerRequest :definition-)
      "float64[3] point
float64[3] tcpPmarker
float64[3] tcpRmarker
---
int8 ok
")

(setf (get mp_mini_picker::moveToPointMarkerResponse :md5sum-) "861274133f98039e263a7bec0ea74796")
(setf (get mp_mini_picker::moveToPointMarkerResponse :datatype-) "mp_mini_picker/moveToPointMarkerResponse")
(setf (get mp_mini_picker::moveToPointMarkerResponse :definition-)
      "float64[3] point
float64[3] tcpPmarker
float64[3] tcpRmarker
---
int8 ok
")



(provide :mp_mini_picker/moveToPointMarker "861274133f98039e263a7bec0ea74796")


