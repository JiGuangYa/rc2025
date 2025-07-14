;; Auto-generated. Do not edit!


(when (boundp 'user_msgs::ObjDets)
  (if (not (find-package "USER_MSGS"))
    (make-package "USER_MSGS"))
  (shadow 'ObjDets (find-package "USER_MSGS")))
(unless (find-package "USER_MSGS::OBJDETS")
  (make-package "USER_MSGS::OBJDETS"))

(in-package "ROS")
;;//! \htmlinclude ObjDets.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass user_msgs::ObjDets
  :super ros::object
  :slots (_header _fps _num _dets ))

(defmethod user_msgs::ObjDets
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:fps __fps) 0)
    ((:num __num) 0)
    ((:dets __dets) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _fps (round __fps))
   (setq _num (round __num))
   (setq _dets __dets)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:fps
   (&optional __fps)
   (if __fps (setq _fps __fps)) _fps)
  (:num
   (&optional __num)
   (if __num (setq _num __num)) _num)
  (:dets
   (&rest __dets)
   (if (keywordp (car __dets))
       (send* _dets __dets)
     (progn
       (if __dets (setq _dets (car __dets)))
       _dets)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int16 _fps
    2
    ;; int16 _num
    2
    ;; user_msgs/ObjDet[] _dets
    (apply #'+ (send-all _dets :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int16 _fps
       (write-word _fps s)
     ;; int16 _num
       (write-word _num s)
     ;; user_msgs/ObjDet[] _dets
     (write-long (length _dets) s)
     (dolist (elem _dets)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int16 _fps
     (setq _fps (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _num
     (setq _num (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; user_msgs/ObjDet[] _dets
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _dets (let (r) (dotimes (i n) (push (instance user_msgs::ObjDet :init) r)) r))
     (dolist (elem- _dets)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get user_msgs::ObjDets :md5sum-) "1c6e1901031d8b3566cf3e9e9d7037d6")
(setf (get user_msgs::ObjDets :datatype-) "user_msgs/ObjDets")
(setf (get user_msgs::ObjDets :definition-)
      "Header header

int16 fps

int16 num

ObjDet[] dets



================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: user_msgs/ObjDet
uint16 index

uint8 class_id

string label

float64 score

ObjBbox bbox

geometry_msgs/Pose pose

================================================================================
MSG: user_msgs/ObjBbox
float64 x1
float64 y1
float64 x2
float64 y2

float64 x
float64 y
float64 w
float64 h

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :user_msgs/ObjDets "1c6e1901031d8b3566cf3e9e9d7037d6")


