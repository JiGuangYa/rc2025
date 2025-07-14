;; Auto-generated. Do not edit!


(when (boundp 'user_msgs::ObjDet)
  (if (not (find-package "USER_MSGS"))
    (make-package "USER_MSGS"))
  (shadow 'ObjDet (find-package "USER_MSGS")))
(unless (find-package "USER_MSGS::OBJDET")
  (make-package "USER_MSGS::OBJDET"))

(in-package "ROS")
;;//! \htmlinclude ObjDet.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass user_msgs::ObjDet
  :super ros::object
  :slots (_index _class_id _label _score _bbox _pose ))

(defmethod user_msgs::ObjDet
  (:init
   (&key
    ((:index __index) 0)
    ((:class_id __class_id) 0)
    ((:label __label) "")
    ((:score __score) 0.0)
    ((:bbox __bbox) (instance user_msgs::ObjBbox :init))
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _index (round __index))
   (setq _class_id (round __class_id))
   (setq _label (string __label))
   (setq _score (float __score))
   (setq _bbox __bbox)
   (setq _pose __pose)
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:class_id
   (&optional __class_id)
   (if __class_id (setq _class_id __class_id)) _class_id)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:score
   (&optional __score)
   (if __score (setq _score __score)) _score)
  (:bbox
   (&rest __bbox)
   (if (keywordp (car __bbox))
       (send* _bbox __bbox)
     (progn
       (if __bbox (setq _bbox (car __bbox)))
       _bbox)))
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; uint16 _index
    2
    ;; uint8 _class_id
    1
    ;; string _label
    4 (length _label)
    ;; float64 _score
    8
    ;; user_msgs/ObjBbox _bbox
    (send _bbox :serialization-length)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _index
       (write-word _index s)
     ;; uint8 _class_id
       (write-byte _class_id s)
     ;; string _label
       (write-long (length _label) s) (princ _label s)
     ;; float64 _score
       (sys::poke _score (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; user_msgs/ObjBbox _bbox
       (send _bbox :serialize s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _index
     (setq _index (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _class_id
     (setq _class_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _score
     (setq _score (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; user_msgs/ObjBbox _bbox
     (send _bbox :deserialize buf ptr-) (incf ptr- (send _bbox :serialization-length))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get user_msgs::ObjDet :md5sum-) "38284f8c06edb8f35a2f0a300b2226fc")
(setf (get user_msgs::ObjDet :datatype-) "user_msgs/ObjDet")
(setf (get user_msgs::ObjDet :definition-)
      "uint16 index

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



(provide :user_msgs/ObjDet "38284f8c06edb8f35a2f0a300b2226fc")


