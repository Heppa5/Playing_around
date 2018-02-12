
(cl:in-package :asdf)

(defsystem "mp_mini_picker-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "currentQ" :depends-on ("_package_currentQ"))
    (:file "_package_currentQ" :depends-on ("_package"))
    (:file "moveToPointMarker" :depends-on ("_package_moveToPointMarker"))
    (:file "_package_moveToPointMarker" :depends-on ("_package"))
    (:file "moveToPointTcp" :depends-on ("_package_moveToPointTcp"))
    (:file "_package_moveToPointTcp" :depends-on ("_package"))
    (:file "moveToPoseMarker" :depends-on ("_package_moveToPoseMarker"))
    (:file "_package_moveToPoseMarker" :depends-on ("_package"))
    (:file "moveToPoseTcp" :depends-on ("_package_moveToPoseTcp"))
    (:file "_package_moveToPoseTcp" :depends-on ("_package"))
    (:file "moveToQ" :depends-on ("_package_moveToQ"))
    (:file "_package_moveToQ" :depends-on ("_package"))
  ))