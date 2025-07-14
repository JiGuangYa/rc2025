; Auto-generated. Do not edit!


(cl:in-package user_msgs-msg)


;//! \htmlinclude CannonStatus.msg.html

(cl:defclass <CannonStatus> (roslisp-msg-protocol:ros-message)
  ((bullet_num
    :reader bullet_num
    :initarg :bullet_num
    :type cl:fixnum
    :initform 0)
   (capacity
    :reader capacity
    :initarg :capacity
    :type cl:fixnum
    :initform 0)
   (reload_time
    :reader reload_time
    :initarg :reload_time
    :type cl:float
    :initform 0.0)
   (rdy2fire
    :reader rdy2fire
    :initarg :rdy2fire
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CannonStatus (<CannonStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CannonStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CannonStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name user_msgs-msg:<CannonStatus> is deprecated: use user_msgs-msg:CannonStatus instead.")))

(cl:ensure-generic-function 'bullet_num-val :lambda-list '(m))
(cl:defmethod bullet_num-val ((m <CannonStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:bullet_num-val is deprecated.  Use user_msgs-msg:bullet_num instead.")
  (bullet_num m))

(cl:ensure-generic-function 'capacity-val :lambda-list '(m))
(cl:defmethod capacity-val ((m <CannonStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:capacity-val is deprecated.  Use user_msgs-msg:capacity instead.")
  (capacity m))

(cl:ensure-generic-function 'reload_time-val :lambda-list '(m))
(cl:defmethod reload_time-val ((m <CannonStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:reload_time-val is deprecated.  Use user_msgs-msg:reload_time instead.")
  (reload_time m))

(cl:ensure-generic-function 'rdy2fire-val :lambda-list '(m))
(cl:defmethod rdy2fire-val ((m <CannonStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:rdy2fire-val is deprecated.  Use user_msgs-msg:rdy2fire instead.")
  (rdy2fire m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CannonStatus>) ostream)
  "Serializes a message object of type '<CannonStatus>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bullet_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'capacity)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'reload_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'rdy2fire) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CannonStatus>) istream)
  "Deserializes a message object of type '<CannonStatus>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bullet_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'capacity)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'reload_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'rdy2fire) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CannonStatus>)))
  "Returns string type for a message object of type '<CannonStatus>"
  "user_msgs/CannonStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CannonStatus)))
  "Returns string type for a message object of type 'CannonStatus"
  "user_msgs/CannonStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CannonStatus>)))
  "Returns md5sum for a message object of type '<CannonStatus>"
  "e31433468f551973da97c9147997bd00")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CannonStatus)))
  "Returns md5sum for a message object of type 'CannonStatus"
  "e31433468f551973da97c9147997bd00")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CannonStatus>)))
  "Returns full string definition for message of type '<CannonStatus>"
  (cl:format cl:nil "uint8 bullet_num~%~%uint8 capacity~%float32 reload_time~%~%bool rdy2fire~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CannonStatus)))
  "Returns full string definition for message of type 'CannonStatus"
  (cl:format cl:nil "uint8 bullet_num~%~%uint8 capacity~%float32 reload_time~%~%bool rdy2fire~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CannonStatus>))
  (cl:+ 0
     1
     1
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CannonStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'CannonStatus
    (cl:cons ':bullet_num (bullet_num msg))
    (cl:cons ':capacity (capacity msg))
    (cl:cons ':reload_time (reload_time msg))
    (cl:cons ':rdy2fire (rdy2fire msg))
))
