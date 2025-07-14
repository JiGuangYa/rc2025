; Auto-generated. Do not edit!


(cl:in-package user_msgs-msg)


;//! \htmlinclude GimbalPoseDeg.msg.html

(cl:defclass <GimbalPoseDeg> (roslisp-msg-protocol:ros-message)
  ((roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass GimbalPoseDeg (<GimbalPoseDeg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GimbalPoseDeg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GimbalPoseDeg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name user_msgs-msg:<GimbalPoseDeg> is deprecated: use user_msgs-msg:GimbalPoseDeg instead.")))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <GimbalPoseDeg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:roll-val is deprecated.  Use user_msgs-msg:roll instead.")
  (roll m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <GimbalPoseDeg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:pitch-val is deprecated.  Use user_msgs-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <GimbalPoseDeg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:yaw-val is deprecated.  Use user_msgs-msg:yaw instead.")
  (yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GimbalPoseDeg>) ostream)
  "Serializes a message object of type '<GimbalPoseDeg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GimbalPoseDeg>) istream)
  "Deserializes a message object of type '<GimbalPoseDeg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GimbalPoseDeg>)))
  "Returns string type for a message object of type '<GimbalPoseDeg>"
  "user_msgs/GimbalPoseDeg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalPoseDeg)))
  "Returns string type for a message object of type 'GimbalPoseDeg"
  "user_msgs/GimbalPoseDeg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GimbalPoseDeg>)))
  "Returns md5sum for a message object of type '<GimbalPoseDeg>"
  "c66f4de7f99199dd8e863fffbef112ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GimbalPoseDeg)))
  "Returns md5sum for a message object of type 'GimbalPoseDeg"
  "c66f4de7f99199dd8e863fffbef112ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GimbalPoseDeg>)))
  "Returns full string definition for message of type '<GimbalPoseDeg>"
  (cl:format cl:nil "# °~%float32 roll~%float32 pitch~%float32 yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GimbalPoseDeg)))
  "Returns full string definition for message of type 'GimbalPoseDeg"
  (cl:format cl:nil "# °~%float32 roll~%float32 pitch~%float32 yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GimbalPoseDeg>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GimbalPoseDeg>))
  "Converts a ROS message object to a list"
  (cl:list 'GimbalPoseDeg
    (cl:cons ':roll (roll msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
))
