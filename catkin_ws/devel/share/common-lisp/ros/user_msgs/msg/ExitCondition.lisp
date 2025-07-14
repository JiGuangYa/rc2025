; Auto-generated. Do not edit!


(cl:in-package user_msgs-msg)


;//! \htmlinclude ExitCondition.msg.html

(cl:defclass <ExitCondition> (roslisp-msg-protocol:ros-message)
  ((exit_l
    :reader exit_l
    :initarg :exit_l
    :type cl:float
    :initform 0.0)
   (exit_a
    :reader exit_a
    :initarg :exit_a
    :type cl:float
    :initform 0.0)
   (arrive_cnt
    :reader arrive_cnt
    :initarg :arrive_cnt
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ExitCondition (<ExitCondition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExitCondition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExitCondition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name user_msgs-msg:<ExitCondition> is deprecated: use user_msgs-msg:ExitCondition instead.")))

(cl:ensure-generic-function 'exit_l-val :lambda-list '(m))
(cl:defmethod exit_l-val ((m <ExitCondition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:exit_l-val is deprecated.  Use user_msgs-msg:exit_l instead.")
  (exit_l m))

(cl:ensure-generic-function 'exit_a-val :lambda-list '(m))
(cl:defmethod exit_a-val ((m <ExitCondition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:exit_a-val is deprecated.  Use user_msgs-msg:exit_a instead.")
  (exit_a m))

(cl:ensure-generic-function 'arrive_cnt-val :lambda-list '(m))
(cl:defmethod arrive_cnt-val ((m <ExitCondition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:arrive_cnt-val is deprecated.  Use user_msgs-msg:arrive_cnt instead.")
  (arrive_cnt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExitCondition>) ostream)
  "Serializes a message object of type '<ExitCondition>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'exit_l))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'exit_a))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'arrive_cnt)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExitCondition>) istream)
  "Deserializes a message object of type '<ExitCondition>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'exit_l) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'exit_a) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'arrive_cnt)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExitCondition>)))
  "Returns string type for a message object of type '<ExitCondition>"
  "user_msgs/ExitCondition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExitCondition)))
  "Returns string type for a message object of type 'ExitCondition"
  "user_msgs/ExitCondition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExitCondition>)))
  "Returns md5sum for a message object of type '<ExitCondition>"
  "7aec7e7f502d48d3f927e4aa6e30673c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExitCondition)))
  "Returns md5sum for a message object of type 'ExitCondition"
  "7aec7e7f502d48d3f927e4aa6e30673c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExitCondition>)))
  "Returns full string definition for message of type '<ExitCondition>"
  (cl:format cl:nil "float64 exit_l  # m~%float64 exit_a  # °~%uint8 arrive_cnt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExitCondition)))
  "Returns full string definition for message of type 'ExitCondition"
  (cl:format cl:nil "float64 exit_l  # m~%float64 exit_a  # °~%uint8 arrive_cnt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExitCondition>))
  (cl:+ 0
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExitCondition>))
  "Converts a ROS message object to a list"
  (cl:list 'ExitCondition
    (cl:cons ':exit_l (exit_l msg))
    (cl:cons ':exit_a (exit_a msg))
    (cl:cons ':arrive_cnt (arrive_cnt msg))
))
