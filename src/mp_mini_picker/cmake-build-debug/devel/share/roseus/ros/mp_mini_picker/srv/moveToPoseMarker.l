;; Auto-generated. Do not edit!


(when (boundp 'mp_mini_picker::moveToPoseMarker)
  (if (not (find-package "MP_MINI_PICKER"))
    (make-package "MP_MINI_PICKER"))
  (shadow 'moveToPoseMarker (find-package "MP_MINI_PICKER")))
(unless (find-package "MP_MINI_PICKER::MOVETOPOSEMARKER")
  (make-package "MP_MINI_PICKER::MOVETOPOSEMARKER"))
(unless (find-package "MP_MINI_PICKER::MOVETOPOSEMARKERREQUEST")
  (make-package "MP_MINI_PICKER::MOVETOPOSEMARKERREQUEST"))
(unless (find-package "MP_MINI_PICKER::MOVETOPOSEMARKERRESPONSE")
  (make-package "MP_MINI_PICKER::MOVETOPOSEMARKERRESPONSE"))

(in-package "ROS")





(defclass mp_mini_picker::moveToPoseMarkerRequest
  :super ros::object
  :slots (_pose _tcpPmarker _tcpRmarker ))

(defmethod mp_mini_picker::moveToPoseMarkerRequest
  (:init
   (&key
    ((:pose __pose) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:tcpPmarker __tcpPmarker) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:tcpRmarker __tcpRmarker) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _pose __pose)
   (setq _tcpPmarker __tcpPmarker)
   (setq _tcpRmarker __tcpRmarker)
   self)
  (:pose
   (&optional __pose)
   (if __pose (setq _pose __pose)) _pose)
  (:tcpPmarker
   (&optional __tcpPmarker)
   (if __tcpPmarker (setq _tcpPmarker __tcpPmarker)) _tcpPmarker)
  (:tcpRmarker
   (&optional __tcpRmarker)
   (if __tcpRmarker (setq _tcpRmarker __tcpRmarker)) _tcpRmarker)
  (:serialization-length
   ()
   (+
    ;; float64[6] _pose
    (* 8    6)
    ;; float64[3] _tcpPmarker
    (* 8    3)
    ;; float64[3] _tcpRmarker
    (* 8    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _pose
     (dotimes (i 6)
       (sys::poke (elt _pose i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
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
   ;; float64[6] _pose
   (dotimes (i (length _pose))
     (setf (elt _pose i) (sys::peek buf ptr- :double)) (incf ptr- 8)
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

(defclass mp_mini_picker::moveToPoseMarkerResponse
  :super ros::object
  :slots (_ok ))

(defmethod mp_mini_picker::moveToPoseMarkerResponse
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

(defclass mp_mini_picker::moveToPoseMarker
  :super ros::object
  :slots ())

(setf (get mp_mini_picker::moveToPoseMarker :md5sum-) "90ff3d42ea6ed49171a1cd36008690f4")
(setf (get mp_mini_picker::moveToPoseMarker :datatype-) "mp_mini_picker/moveToPoseMarker")
(setf (get mp_mini_picker::moveToPoseMarker :request) mp_mini_picker::moveToPoseMarkerRequest)
(setf (get mp_mini_picker::moveToPoseMarker :response) mp_mini_picker::moveToPoseMarkerResponse)

(defmethod mp_mini_picker::moveToPoseMarkerRequest
  (:response () (instance mp_mini_picker::moveToPoseMarkerResponse :init)))

(setf (get mp_mini_picker::moveToPoseMarkerRequest :md5sum-) "90ff3d42ea6ed49171a1cd36008690f4")
(setf (get mp_mini_picker::moveToPoseMarkerRequest :datatype-) "mp_mini_picker/moveToPoseMarkerRequest")
(setf (get mp_mini_picker::moveToPoseMarkerRequest :definition-)
      "float64[6] pose
float64[3] tcpPmarker
float64[3] tcpRmarker
---
int8 ok
")

(setf (get mp_mini_picker::moveToPoseMarkerResponse :md5sum-) "90ff3d42ea6ed49171a1cd36008690f4")
(setf (get mp_mini_picker::moveToPoseMarkerResponse :datatype-) "mp_mini_picker/moveToPoseMarkerResponse")
(setf (get mp_mini_picker::moveToPoseMarkerResponse :definition-)
      "float64[6] pose
float64[3] tcpPmarker
float64[3] tcpRmarker
---
int8 ok
")



(provide :mp_mini_picker/moveToPoseMarker "90ff3d42ea6ed49171a1cd36008690f4")


