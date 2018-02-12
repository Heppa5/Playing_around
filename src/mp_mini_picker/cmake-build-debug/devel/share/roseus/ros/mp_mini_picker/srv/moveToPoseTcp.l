;; Auto-generated. Do not edit!


(when (boundp 'mp_mini_picker::moveToPoseTcp)
  (if (not (find-package "MP_MINI_PICKER"))
    (make-package "MP_MINI_PICKER"))
  (shadow 'moveToPoseTcp (find-package "MP_MINI_PICKER")))
(unless (find-package "MP_MINI_PICKER::MOVETOPOSETCP")
  (make-package "MP_MINI_PICKER::MOVETOPOSETCP"))
(unless (find-package "MP_MINI_PICKER::MOVETOPOSETCPREQUEST")
  (make-package "MP_MINI_PICKER::MOVETOPOSETCPREQUEST"))
(unless (find-package "MP_MINI_PICKER::MOVETOPOSETCPRESPONSE")
  (make-package "MP_MINI_PICKER::MOVETOPOSETCPRESPONSE"))

(in-package "ROS")





(defclass mp_mini_picker::moveToPoseTcpRequest
  :super ros::object
  :slots (_pose ))

(defmethod mp_mini_picker::moveToPoseTcpRequest
  (:init
   (&key
    ((:pose __pose) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _pose __pose)
   self)
  (:pose
   (&optional __pose)
   (if __pose (setq _pose __pose)) _pose)
  (:serialization-length
   ()
   (+
    ;; float64[6] _pose
    (* 8    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _pose
     (dotimes (i 6)
       (sys::poke (elt _pose i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _pose
   (dotimes (i (length _pose))
     (setf (elt _pose i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass mp_mini_picker::moveToPoseTcpResponse
  :super ros::object
  :slots (_ok ))

(defmethod mp_mini_picker::moveToPoseTcpResponse
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

(defclass mp_mini_picker::moveToPoseTcp
  :super ros::object
  :slots ())

(setf (get mp_mini_picker::moveToPoseTcp :md5sum-) "c25f95a4c81298b09162de739af2fd0e")
(setf (get mp_mini_picker::moveToPoseTcp :datatype-) "mp_mini_picker/moveToPoseTcp")
(setf (get mp_mini_picker::moveToPoseTcp :request) mp_mini_picker::moveToPoseTcpRequest)
(setf (get mp_mini_picker::moveToPoseTcp :response) mp_mini_picker::moveToPoseTcpResponse)

(defmethod mp_mini_picker::moveToPoseTcpRequest
  (:response () (instance mp_mini_picker::moveToPoseTcpResponse :init)))

(setf (get mp_mini_picker::moveToPoseTcpRequest :md5sum-) "c25f95a4c81298b09162de739af2fd0e")
(setf (get mp_mini_picker::moveToPoseTcpRequest :datatype-) "mp_mini_picker/moveToPoseTcpRequest")
(setf (get mp_mini_picker::moveToPoseTcpRequest :definition-)
      "float64[6] pose
---
int8 ok
")

(setf (get mp_mini_picker::moveToPoseTcpResponse :md5sum-) "c25f95a4c81298b09162de739af2fd0e")
(setf (get mp_mini_picker::moveToPoseTcpResponse :datatype-) "mp_mini_picker/moveToPoseTcpResponse")
(setf (get mp_mini_picker::moveToPoseTcpResponse :definition-)
      "float64[6] pose
---
int8 ok
")



(provide :mp_mini_picker/moveToPoseTcp "c25f95a4c81298b09162de739af2fd0e")


