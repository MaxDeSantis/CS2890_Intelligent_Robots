;; Auto-generated. Do not edit!


(when (boundp 'ball_finder::BallLocation)
  (if (not (find-package "BALL_FINDER"))
    (make-package "BALL_FINDER"))
  (shadow 'BallLocation (find-package "BALL_FINDER")))
(unless (find-package "BALL_FINDER::BALLLOCATION")
  (make-package "BALL_FINDER::BALLLOCATION"))

(in-package "ROS")
;;//! \htmlinclude BallLocation.msg.html


(defclass ball_finder::BallLocation
  :super ros::object
  :slots (_bearing _distance ))

(defmethod ball_finder::BallLocation
  (:init
   (&key
    ((:bearing __bearing) 0)
    ((:distance __distance) 0)
    )
   (send-super :init)
   (setq _bearing (round __bearing))
   (setq _distance (round __distance))
   self)
  (:bearing
   (&optional __bearing)
   (if __bearing (setq _bearing __bearing)) _bearing)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; int32 _bearing
    4
    ;; int32 _distance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _bearing
       (write-long _bearing s)
     ;; int32 _distance
       (write-long _distance s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _bearing
     (setq _bearing (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _distance
     (setq _distance (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ball_finder::BallLocation :md5sum-) "09f1160bffd7c7871c2baa2dd0283a30")
(setf (get ball_finder::BallLocation :datatype-) "ball_finder/BallLocation")
(setf (get ball_finder::BallLocation :definition-)
      "int32 bearing
int32 distance
")



(provide :ball_finder/BallLocation "09f1160bffd7c7871c2baa2dd0283a30")


