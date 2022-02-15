
(cl:in-package :asdf)

(defsystem "ball_finder-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "BallLocation" :depends-on ("_package_BallLocation"))
    (:file "_package_BallLocation" :depends-on ("_package"))
  ))