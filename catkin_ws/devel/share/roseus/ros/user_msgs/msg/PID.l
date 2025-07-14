;; Auto-generated. Do not edit!


(when (boundp 'user_msgs::PID)
  (if (not (find-package "USER_MSGS"))
    (make-package "USER_MSGS"))
  (shadow 'PID (find-package "USER_MSGS")))
(unless (find-package "USER_MSGS::PID")
  (make-package "USER_MSGS::PID"))

(in-package "ROS")
;;//! \htmlinclude PID.msg.html


(defclass user_msgs::PID
  :super ros::object
  :slots (_p _i _d _i_activate _out_thresh ))

(defmethod user_msgs::PID
  (:init
   (&key
    ((:p __p) 0.0)
    ((:i __i) 0.0)
    ((:d __d) 0.0)
    ((:i_activate __i_activate) 0.0)
    ((:out_thresh __out_thresh) 0.0)
    )
   (send-super :init)
   (setq _p (float __p))
   (setq _i (float __i))
   (setq _d (float __d))
   (setq _i_activate (float __i_activate))
   (setq _out_thresh (float __out_thresh))
   self)
  (:p
   (&optional __p)
   (if __p (setq _p __p)) _p)
  (:i
   (&optional __i)
   (if __i (setq _i __i)) _i)
  (:d
   (&optional __d)
   (if __d (setq _d __d)) _d)
  (:i_activate
   (&optional __i_activate)
   (if __i_activate (setq _i_activate __i_activate)) _i_activate)
  (:out_thresh
   (&optional __out_thresh)
   (if __out_thresh (setq _out_thresh __out_thresh)) _out_thresh)
  (:serialization-length
   ()
   (+
    ;; float32 _p
    4
    ;; float32 _i
    4
    ;; float32 _d
    4
    ;; float32 _i_activate
    4
    ;; float32 _out_thresh
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _p
       (sys::poke _p (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _i
       (sys::poke _i (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _d
       (sys::poke _d (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _i_activate
       (sys::poke _i_activate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _out_thresh
       (sys::poke _out_thresh (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _p
     (setq _p (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _i
     (setq _i (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _d
     (setq _d (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _i_activate
     (setq _i_activate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _out_thresh
     (setq _out_thresh (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get user_msgs::PID :md5sum-) "a7a91d2c8b17a0071e3c8256ef5a2554")
(setf (get user_msgs::PID :datatype-) "user_msgs/PID")
(setf (get user_msgs::PID :definition-)
      "float32 p
float32 i
float32 d
float32 i_activate
float32 out_thresh

")



(provide :user_msgs/PID "a7a91d2c8b17a0071e3c8256ef5a2554")


