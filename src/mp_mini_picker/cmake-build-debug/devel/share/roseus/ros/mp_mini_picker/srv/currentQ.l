;; Auto-generated. Do not edit!


(when (boundp 'mp_mini_picker::currentQ)
  (if (not (find-package "MP_MINI_PICKER"))
    (make-package "MP_MINI_PICKER"))
  (shadow 'currentQ (find-package "MP_MINI_PICKER")))
(unless (find-package "MP_MINI_PICKER::CURRENTQ")
  (make-package "MP_MINI_PICKER::CURRENTQ"))
(unless (find-package "MP_MINI_PICKER::CURRENTQREQUEST")
  (make-package "MP_MINI_PICKER::CURRENTQREQUEST"))
(unless (find-package "MP_MINI_PICKER::CURRENTQRESPONSE")
  (make-package "MP_MINI_PICKER::CURRENTQRESPONSE"))

(in-package "ROS")





(defclass mp_mini_picker::currentQRequest
  :super ros::object
  :slots (_getQ ))

(defmethod mp_mini_picker::currentQRequest
  (:init
   (&key
    ((:getQ __getQ) 0)
    )
   (send-super :init)
   (setq _getQ (round __getQ))
   self)
  (:getQ
   (&optional __getQ)
   (if __getQ (setq _getQ __getQ)) _getQ)
  (:serialization-length
   ()
   (+
    ;; int8 _getQ
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _getQ
       (write-byte _getQ s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _getQ
     (setq _getQ (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _getQ 127) (setq _getQ (- _getQ 256)))
   ;;
   self)
  )

(defclass mp_mini_picker::currentQResponse
  :super ros::object
  :slots (_Q ))

(defmethod mp_mini_picker::currentQResponse
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

(defclass mp_mini_picker::currentQ
  :super ros::object
  :slots ())

(setf (get mp_mini_picker::currentQ :md5sum-) "0abad29c7af93c5a38bad143c421d80d")
(setf (get mp_mini_picker::currentQ :datatype-) "mp_mini_picker/currentQ")
(setf (get mp_mini_picker::currentQ :request) mp_mini_picker::currentQRequest)
(setf (get mp_mini_picker::currentQ :response) mp_mini_picker::currentQResponse)

(defmethod mp_mini_picker::currentQRequest
  (:response () (instance mp_mini_picker::currentQResponse :init)))

(setf (get mp_mini_picker::currentQRequest :md5sum-) "0abad29c7af93c5a38bad143c421d80d")
(setf (get mp_mini_picker::currentQRequest :datatype-) "mp_mini_picker/currentQRequest")
(setf (get mp_mini_picker::currentQRequest :definition-)
      "int8 getQ
---
float64[6] Q

")

(setf (get mp_mini_picker::currentQResponse :md5sum-) "0abad29c7af93c5a38bad143c421d80d")
(setf (get mp_mini_picker::currentQResponse :datatype-) "mp_mini_picker/currentQResponse")
(setf (get mp_mini_picker::currentQResponse :definition-)
      "int8 getQ
---
float64[6] Q

")



(provide :mp_mini_picker/currentQ "0abad29c7af93c5a38bad143c421d80d")


