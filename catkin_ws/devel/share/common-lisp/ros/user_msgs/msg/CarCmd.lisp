; Auto-generated. Do not edit!


(cl:in-package user_msgs-msg)


;//! \htmlinclude CarCmd.msg.html

(cl:defclass <CarCmd> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CarCmd (<CarCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CarCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CarCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name user_msgs-msg:<CarCmd> is deprecated: use user_msgs-msg:CarCmd instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <CarCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:cmd-val is deprecated.  Use user_msgs-msg:cmd instead.")
  (cmd m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CarCmd>)))
    "Constants for message type '<CarCmd>"
  '((:HOLD . 0)
    (:REMOTE . 1)
    (:NAVIGATION . 2)
    (:ARRIVE . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CarCmd)))
    "Constants for message type 'CarCmd"
  '((:HOLD . 0)
    (:REMOTE . 1)
    (:NAVIGATION . 2)
    (:ARRIVE . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CarCmd>) ostream)
  "Serializes a message object of type '<CarCmd>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CarCmd>) istream)
  "Deserializes a message object of type '<CarCmd>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CarCmd>)))
  "Returns string type for a message object of type '<CarCmd>"
  "user_msgs/CarCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CarCmd)))
  "Returns string type for a message object of type 'CarCmd"
  "user_msgs/CarCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CarCmd>)))
  "Returns md5sum for a message object of type '<CarCmd>"
  "1ce4d776e578234f635535a4932fa551")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CarCmd)))
  "Returns md5sum for a message object of type 'CarCmd"
  "1ce4d776e578234f635535a4932fa551")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CarCmd>)))
  "Returns full string definition for message of type '<CarCmd>"
  (cl:format cl:nil "uint8 cmd~%~%~%# enum cmd~%uint8 HOLD=0~%uint8 REMOTE=1~%uint8 NAVIGATION=2~%uint8 ARRIVE=3~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CarCmd)))
  "Returns full string definition for message of type 'CarCmd"
  (cl:format cl:nil "uint8 cmd~%~%~%# enum cmd~%uint8 HOLD=0~%uint8 REMOTE=1~%uint8 NAVIGATION=2~%uint8 ARRIVE=3~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CarCmd>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CarCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'CarCmd
    (cl:cons ':cmd (cmd msg))
))
