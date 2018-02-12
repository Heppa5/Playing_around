;; Auto-generated. Do not edit!


(when (boundp 'mp_mini_picker::moveToQ)
  (if (not (find-package "MP_MINI_PICKER"))
    (make-package "MP_MINI_PICKER"))
  (shadow 'moveToQ (find-package "MP_MINI_PICKER")))
(unless (find-package "MP_MINI_PICKER::MOVETOQ")
  (make-package "MP_MINI_PICKER::MOVETOQ"))
(unless (find-package "MP_MINI_PICKER::MOVETOQREQUEST")
  (make-package "MP_MINI_PICKER::MOVETOQREQUEST"))
(unless (find-package "MP_MINI_PICKER::MOVETOQRESPONSE")
  (make-package "MP_MINI_PICKER::MOVETOQRESPONSE"))

(in-package "ROS")





(defclass mp_mini_picker::moveToQRequest
  :super ros::object
  :slots (_Q ))

(defmethod mp_mini_picker::moveToQRequest
  (:init
   (&key
    ((:Q __Q) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _Q __Q)
   self)
  (:Q
   (&optional __Q)
   (if __Q (setq _Q __Q)) _Q)
  (:serialization-length
   ()
   (+
    ;; float64[6] _Q
    (* 8    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _Q
     (dotimes (i 6)
       (sys::poke (elt _Q i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _Q
   (dotimes (i (length _Q))
     (setf (elt _Q i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass mp_mini_picker::moveToQResponse
  :super ros::object
  :slots (_ok ))

(defmethod mp_mini_picker::moveToQResponse
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

(defclass mp_mini_picker::moveToQ
  :super ros::object
  :slots ())

(setf (get mp_mini_picker::moveToQ :md5sum-) "1d1efbf9948db38dda60ea143a471260")
(setf (get mp_mini_picker::moveToQ :datatype-) "mp_mini_picker/moveToQ")
(setf (get mp_mini_picker::moveToQ :request) mp_mini_picker::moveToQRequest)
(setf (get mp_mini_picker::moveToQ :response) mp_mini_picker::moveToQResponse)

(defmethod mp_mini_picker::moveToQRequest
  (:response () (instance mp_mini_picker::moveToQResponse :init)))

(setf (get mp_mini_picker::moveToQRequest :md5sum-) "1d1efbf9948db38dda60ea143a471260")
(setf (get mp_mini_picker::moveToQRequest :datatype-) "mp_mini_picker/moveToQRequest")
(setf (get mp_mini_picker::moveToQRequest :definition-)
      "float64[6] Q
---
int8 ok

")

(setf (get mp_mini_picker::moveToQResponse :md5sum-) "1d1efbf9948db38dda60ea143a471260")
(setf (get mp_mini_picker::moveToQResponse :datatype-) "mp_mini_picker/moveToQResponse")
(setf (get mp_mini_picker::moveToQResponse :definition-)
      "float64[6] Q
---
int8 ok

")



(provide :mp_mini_picker/moveToQ "1d1efbf9948db38dda60ea143a471260")


