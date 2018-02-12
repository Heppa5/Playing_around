;; Auto-generated. Do not edit!


(when (boundp 'mp_mini_picker::moveToPointTcp)
  (if (not (find-package "MP_MINI_PICKER"))
    (make-package "MP_MINI_PICKER"))
  (shadow 'moveToPointTcp (find-package "MP_MINI_PICKER")))
(unless (find-package "MP_MINI_PICKER::MOVETOPOINTTCP")
  (make-package "MP_MINI_PICKER::MOVETOPOINTTCP"))
(unless (find-package "MP_MINI_PICKER::MOVETOPOINTTCPREQUEST")
  (make-package "MP_MINI_PICKER::MOVETOPOINTTCPREQUEST"))
(unless (find-package "MP_MINI_PICKER::MOVETOPOINTTCPRESPONSE")
  (make-package "MP_MINI_PICKER::MOVETOPOINTTCPRESPONSE"))

(in-package "ROS")





(defclass mp_mini_picker::moveToPointTcpRequest
  :super ros::object
  :slots (_point ))

(defmethod mp_mini_picker::moveToPointTcpRequest
  (:init
   (&key
    ((:point __point) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _point __point)
   self)
  (:point
   (&optional __point)
   (if __point (setq _point __point)) _point)
  (:serialization-length
   ()
   (+
    ;; float64[3] _point
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
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[3] _point
   (dotimes (i (length _point))
     (setf (elt _point i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass mp_mini_picker::moveToPointTcpResponse
  :super ros::object
  :slots (_ok ))

(defmethod mp_mini_picker::moveToPointTcpResponse
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

(defclass mp_mini_picker::moveToPointTcp
  :super ros::object
  :slots ())

(setf (get mp_mini_picker::moveToPointTcp :md5sum-) "69fd9ff2255d1e9323bb7ce354efb11f")
(setf (get mp_mini_picker::moveToPointTcp :datatype-) "mp_mini_picker/moveToPointTcp")
(setf (get mp_mini_picker::moveToPointTcp :request) mp_mini_picker::moveToPointTcpRequest)
(setf (get mp_mini_picker::moveToPointTcp :response) mp_mini_picker::moveToPointTcpResponse)

(defmethod mp_mini_picker::moveToPointTcpRequest
  (:response () (instance mp_mini_picker::moveToPointTcpResponse :init)))

(setf (get mp_mini_picker::moveToPointTcpRequest :md5sum-) "69fd9ff2255d1e9323bb7ce354efb11f")
(setf (get mp_mini_picker::moveToPointTcpRequest :datatype-) "mp_mini_picker/moveToPointTcpRequest")
(setf (get mp_mini_picker::moveToPointTcpRequest :definition-)
      "float64[3] point
---
int8 ok
")

(setf (get mp_mini_picker::moveToPointTcpResponse :md5sum-) "69fd9ff2255d1e9323bb7ce354efb11f")
(setf (get mp_mini_picker::moveToPointTcpResponse :datatype-) "mp_mini_picker/moveToPointTcpResponse")
(setf (get mp_mini_picker::moveToPointTcpResponse :definition-)
      "float64[3] point
---
int8 ok
")



(provide :mp_mini_picker/moveToPointTcp "69fd9ff2255d1e9323bb7ce354efb11f")


