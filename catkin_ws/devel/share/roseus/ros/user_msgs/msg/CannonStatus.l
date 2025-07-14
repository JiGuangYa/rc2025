;; Auto-generated. Do not edit!


(when (boundp 'user_msgs::CannonStatus)
  (if (not (find-package "USER_MSGS"))
    (make-package "USER_MSGS"))
  (shadow 'CannonStatus (find-package "USER_MSGS")))
(unless (find-package "USER_MSGS::CANNONSTATUS")
  (make-package "USER_MSGS::CANNONSTATUS"))

(in-package "ROS")
;;//! \htmlinclude CannonStatus.msg.html


(defclass user_msgs::CannonStatus
  :super ros::object
  :slots (_bullet_num _capacity _reload_time _rdy2fire ))

(defmethod user_msgs::CannonStatus
  (:init
   (&key
    ((:bullet_num __bullet_num) 0)
    ((:capacity __capacity) 0)
    ((:reload_time __reload_time) 0.0)
    ((:rdy2fire __rdy2fire) nil)
    )
   (send-super :init)
   (setq _bullet_num (round __bullet_num))
   (setq _capacity (round __capacity))
   (setq _reload_time (float __reload_time))
   (setq _rdy2fire __rdy2fire)
   self)
  (:bullet_num
   (&optional __bullet_num)
   (if __bullet_num (setq _bullet_num __bullet_num)) _bullet_num)
  (:capacity
   (&optional __capacity)
   (if __capacity (setq _capacity __capacity)) _capacity)
  (:reload_time
   (&optional __reload_time)
   (if __reload_time (setq _reload_time __reload_time)) _reload_time)
  (:rdy2fire
   (&optional (__rdy2fire :null))
   (if (not (eq __rdy2fire :null)) (setq _rdy2fire __rdy2fire)) _rdy2fire)
  (:serialization-length
   ()
   (+
    ;; uint8 _bullet_num
    1
    ;; uint8 _capacity
    1
    ;; float32 _reload_time
    4
    ;; bool _rdy2fire
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _bullet_num
       (write-byte _bullet_num s)
     ;; uint8 _capacity
       (write-byte _capacity s)
     ;; float32 _reload_time
       (sys::poke _reload_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _rdy2fire
       (if _rdy2fire (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _bullet_num
     (setq _bullet_num (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _capacity
     (setq _capacity (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _reload_time
     (setq _reload_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _rdy2fire
     (setq _rdy2fire (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get user_msgs::CannonStatus :md5sum-) "e31433468f551973da97c9147997bd00")
(setf (get user_msgs::CannonStatus :datatype-) "user_msgs/CannonStatus")
(setf (get user_msgs::CannonStatus :definition-)
      "uint8 bullet_num

uint8 capacity
float32 reload_time

bool rdy2fire

")



(provide :user_msgs/CannonStatus "e31433468f551973da97c9147997bd00")


