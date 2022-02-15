; Auto-generated. Do not edit!


(cl:in-package ball_finder-msg)


;//! \htmlinclude BallLocation.msg.html

(cl:defclass <BallLocation> (roslisp-msg-protocol:ros-message)
  ((bearing
    :reader bearing
    :initarg :bearing
    :type cl:integer
    :initform 0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:integer
    :initform 0))
)

(cl:defclass BallLocation (<BallLocation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BallLocation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BallLocation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ball_finder-msg:<BallLocation> is deprecated: use ball_finder-msg:BallLocation instead.")))

(cl:ensure-generic-function 'bearing-val :lambda-list '(m))
(cl:defmethod bearing-val ((m <BallLocation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ball_finder-msg:bearing-val is deprecated.  Use ball_finder-msg:bearing instead.")
  (bearing m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <BallLocation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ball_finder-msg:distance-val is deprecated.  Use ball_finder-msg:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BallLocation>) ostream)
  "Serializes a message object of type '<BallLocation>"
  (cl:let* ((signed (cl:slot-value msg 'bearing)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'distance)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BallLocation>) istream)
  "Deserializes a message object of type '<BallLocation>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bearing) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'distance) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BallLocation>)))
  "Returns string type for a message object of type '<BallLocation>"
  "ball_finder/BallLocation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BallLocation)))
  "Returns string type for a message object of type 'BallLocation"
  "ball_finder/BallLocation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BallLocation>)))
  "Returns md5sum for a message object of type '<BallLocation>"
  "09f1160bffd7c7871c2baa2dd0283a30")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BallLocation)))
  "Returns md5sum for a message object of type 'BallLocation"
  "09f1160bffd7c7871c2baa2dd0283a30")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BallLocation>)))
  "Returns full string definition for message of type '<BallLocation>"
  (cl:format cl:nil "int32 bearing~%int32 distance~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BallLocation)))
  "Returns full string definition for message of type 'BallLocation"
  (cl:format cl:nil "int32 bearing~%int32 distance~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BallLocation>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BallLocation>))
  "Converts a ROS message object to a list"
  (cl:list 'BallLocation
    (cl:cons ':bearing (bearing msg))
    (cl:cons ':distance (distance msg))
))
