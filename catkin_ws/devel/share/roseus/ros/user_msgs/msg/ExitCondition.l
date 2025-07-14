;; Auto-generated. Do not edit!


(when (boundp 'user_msgs::ExitCondition)
  (if (not (find-package "USER_MSGS"))
    (make-package "USER_MSGS"))
  (shadow 'ExitCondition (find-package "USER_MSGS")))
(unless (find-package "USER_MSGS::EXITCONDITION")
  (make-package "USER_MSGS::EXITCONDITION"))

(in-package "ROS")
;;//! \htmlinclude ExitCondition.msg.html


(defclass user_msgs::ExitCondition
  :super ros::object
  :slots (_exit_l _exit_a _arrive_cnt ))

(defmethod user_msgs::ExitCondition
  (:init
   (&key
    ((:exit_l __exit_l) 0.0)
    ((:exit_a __exit_a) 0.0)
    ((:arrive_cnt __arrive_cnt) 0)
    )
   (send-super :init)
   (setq _exit_l (float __exit_l))
   (setq _exit_a (float __exit_a))
   (setq _arrive_cnt (round __arrive_cnt))
   self)
  (:exit_l
   (&optional __exit_l)
   (if __exit_l (setq _exit_l __exit_l)) _exit_l)
  (:exit_a
   (&optional __exit_a)
   (if __exit_a (setq _exit_a __exit_a)) _exit_a)
  (:arrive_cnt
   (&optional __arrive_cnt)
   (if __arrive_cnt (setq _arrive_cnt __arrive_cnt)) _arrive_cnt)
  (:serialization-length
   ()
   (+
    ;; float64 _exit_l
    8
    ;; float64 _exit_a
    8
    ;; uint8 _arrive_cnt
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _exit_l
       (sys::poke _exit_l (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _exit_a
       (sys::poke _exit_a (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _arrive_cnt
       (write-byte _arrive_cnt s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _exit_l
     (setq _exit_l (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _exit_a
     (setq _exit_a (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _arrive_cnt
     (setq _arrive_cnt (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get user_msgs::ExitCondition :md5sum-) "7aec7e7f502d48d3f927e4aa6e30673c")
(setf (get user_msgs::ExitCondition :datatype-) "user_msgs/ExitCondition")
(setf (get user_msgs::ExitCondition :definition-)
      "float64 exit_l  # m
float64 exit_a  # °
uint8 arrive_cnt

")



(provide :user_msgs/ExitCondition "7aec7e7f502d48d3f927e4aa6e30673c")


