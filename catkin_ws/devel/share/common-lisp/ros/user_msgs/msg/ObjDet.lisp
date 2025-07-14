; Auto-generated. Do not edit!


(cl:in-package user_msgs-msg)


;//! \htmlinclude ObjDet.msg.html

(cl:defclass <ObjDet> (roslisp-msg-protocol:ros-message)
  ((index
    :reader index
    :initarg :index
    :type cl:fixnum
    :initform 0)
   (class_id
    :reader class_id
    :initarg :class_id
    :type cl:fixnum
    :initform 0)
   (label
    :reader label
    :initarg :label
    :type cl:string
    :initform "")
   (score
    :reader score
    :initarg :score
    :type cl:float
    :initform 0.0)
   (bbox
    :reader bbox
    :initarg :bbox
    :type user_msgs-msg:ObjBbox
    :initform (cl:make-instance 'user_msgs-msg:ObjBbox))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass ObjDet (<ObjDet>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjDet>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjDet)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name user_msgs-msg:<ObjDet> is deprecated: use user_msgs-msg:ObjDet instead.")))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <ObjDet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:index-val is deprecated.  Use user_msgs-msg:index instead.")
  (index m))

(cl:ensure-generic-function 'class_id-val :lambda-list '(m))
(cl:defmethod class_id-val ((m <ObjDet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:class_id-val is deprecated.  Use user_msgs-msg:class_id instead.")
  (class_id m))

(cl:ensure-generic-function 'label-val :lambda-list '(m))
(cl:defmethod label-val ((m <ObjDet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:label-val is deprecated.  Use user_msgs-msg:label instead.")
  (label m))

(cl:ensure-generic-function 'score-val :lambda-list '(m))
(cl:defmethod score-val ((m <ObjDet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:score-val is deprecated.  Use user_msgs-msg:score instead.")
  (score m))

(cl:ensure-generic-function 'bbox-val :lambda-list '(m))
(cl:defmethod bbox-val ((m <ObjDet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:bbox-val is deprecated.  Use user_msgs-msg:bbox instead.")
  (bbox m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <ObjDet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:pose-val is deprecated.  Use user_msgs-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjDet>) ostream)
  "Serializes a message object of type '<ObjDet>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'class_id)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'label))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'label))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'score))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bbox) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjDet>) istream)
  "Deserializes a message object of type '<ObjDet>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'class_id)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'label) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'label) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'score) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bbox) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjDet>)))
  "Returns string type for a message object of type '<ObjDet>"
  "user_msgs/ObjDet")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjDet)))
  "Returns string type for a message object of type 'ObjDet"
  "user_msgs/ObjDet")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjDet>)))
  "Returns md5sum for a message object of type '<ObjDet>"
  "38284f8c06edb8f35a2f0a300b2226fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjDet)))
  "Returns md5sum for a message object of type 'ObjDet"
  "38284f8c06edb8f35a2f0a300b2226fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjDet>)))
  "Returns full string definition for message of type '<ObjDet>"
  (cl:format cl:nil "uint16 index~%~%uint8 class_id~%~%string label~%~%float64 score~%~%ObjBbox bbox~%~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: user_msgs/ObjBbox~%float64 x1~%float64 y1~%float64 x2~%float64 y2~%~%float64 x~%float64 y~%float64 w~%float64 h~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjDet)))
  "Returns full string definition for message of type 'ObjDet"
  (cl:format cl:nil "uint16 index~%~%uint8 class_id~%~%string label~%~%float64 score~%~%ObjBbox bbox~%~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: user_msgs/ObjBbox~%float64 x1~%float64 y1~%float64 x2~%float64 y2~%~%float64 x~%float64 y~%float64 w~%float64 h~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjDet>))
  (cl:+ 0
     2
     1
     4 (cl:length (cl:slot-value msg 'label))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bbox))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjDet>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjDet
    (cl:cons ':index (index msg))
    (cl:cons ':class_id (class_id msg))
    (cl:cons ':label (label msg))
    (cl:cons ':score (score msg))
    (cl:cons ':bbox (bbox msg))
    (cl:cons ':pose (pose msg))
))
