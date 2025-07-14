; Auto-generated. Do not edit!


(cl:in-package user_msgs-msg)


;//! \htmlinclude ObjDets.msg.html

(cl:defclass <ObjDets> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (fps
    :reader fps
    :initarg :fps
    :type cl:fixnum
    :initform 0)
   (num
    :reader num
    :initarg :num
    :type cl:fixnum
    :initform 0)
   (dets
    :reader dets
    :initarg :dets
    :type (cl:vector user_msgs-msg:ObjDet)
   :initform (cl:make-array 0 :element-type 'user_msgs-msg:ObjDet :initial-element (cl:make-instance 'user_msgs-msg:ObjDet))))
)

(cl:defclass ObjDets (<ObjDets>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjDets>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjDets)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name user_msgs-msg:<ObjDets> is deprecated: use user_msgs-msg:ObjDets instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObjDets>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:header-val is deprecated.  Use user_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'fps-val :lambda-list '(m))
(cl:defmethod fps-val ((m <ObjDets>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:fps-val is deprecated.  Use user_msgs-msg:fps instead.")
  (fps m))

(cl:ensure-generic-function 'num-val :lambda-list '(m))
(cl:defmethod num-val ((m <ObjDets>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:num-val is deprecated.  Use user_msgs-msg:num instead.")
  (num m))

(cl:ensure-generic-function 'dets-val :lambda-list '(m))
(cl:defmethod dets-val ((m <ObjDets>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_msgs-msg:dets-val is deprecated.  Use user_msgs-msg:dets instead.")
  (dets m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjDets>) ostream)
  "Serializes a message object of type '<ObjDets>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'fps)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'dets))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'dets))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjDets>) istream)
  "Deserializes a message object of type '<ObjDets>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fps) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'dets) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'dets)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'user_msgs-msg:ObjDet))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjDets>)))
  "Returns string type for a message object of type '<ObjDets>"
  "user_msgs/ObjDets")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjDets)))
  "Returns string type for a message object of type 'ObjDets"
  "user_msgs/ObjDets")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjDets>)))
  "Returns md5sum for a message object of type '<ObjDets>"
  "1c6e1901031d8b3566cf3e9e9d7037d6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjDets)))
  "Returns md5sum for a message object of type 'ObjDets"
  "1c6e1901031d8b3566cf3e9e9d7037d6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjDets>)))
  "Returns full string definition for message of type '<ObjDets>"
  (cl:format cl:nil "Header header~%~%int16 fps~%~%int16 num~%~%ObjDet[] dets~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: user_msgs/ObjDet~%uint16 index~%~%uint8 class_id~%~%string label~%~%float64 score~%~%ObjBbox bbox~%~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: user_msgs/ObjBbox~%float64 x1~%float64 y1~%float64 x2~%float64 y2~%~%float64 x~%float64 y~%float64 w~%float64 h~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjDets)))
  "Returns full string definition for message of type 'ObjDets"
  (cl:format cl:nil "Header header~%~%int16 fps~%~%int16 num~%~%ObjDet[] dets~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: user_msgs/ObjDet~%uint16 index~%~%uint8 class_id~%~%string label~%~%float64 score~%~%ObjBbox bbox~%~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: user_msgs/ObjBbox~%float64 x1~%float64 y1~%float64 x2~%float64 y2~%~%float64 x~%float64 y~%float64 w~%float64 h~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjDets>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'dets) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjDets>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjDets
    (cl:cons ':header (header msg))
    (cl:cons ':fps (fps msg))
    (cl:cons ':num (num msg))
    (cl:cons ':dets (dets msg))
))
