;; Auto-generated. Do not edit!


(when (boundp 'user_msgs::CarCmd)
  (if (not (find-package "USER_MSGS"))
    (make-package "USER_MSGS"))
  (shadow 'CarCmd (find-package "USER_MSGS")))
(unless (find-package "USER_MSGS::CARCMD")
  (make-package "USER_MSGS::CARCMD"))

(in-package "ROS")
;;//! \htmlinclude CarCmd.msg.html


(intern "*HOLD*" (find-package "USER_MSGS::CARCMD"))
(shadow '*HOLD* (find-package "USER_MSGS::CARCMD"))
(defconstant user_msgs::CarCmd::*HOLD* 0)
(intern "*REMOTE*" (find-package "USER_MSGS::CARCMD"))
(shadow '*REMOTE* (find-package "USER_MSGS::CARCMD"))
(defconstant user_msgs::CarCmd::*REMOTE* 1)
(intern "*NAVIGATION*" (find-package "USER_MSGS::CARCMD"))
(shadow '*NAVIGATION* (find-package "USER_MSGS::CARCMD"))
(defconstant user_msgs::CarCmd::*NAVIGATION* 2)
(intern "*ARRIVE*" (find-package "USER_MSGS::CARCMD"))
(shadow '*ARRIVE* (find-package "USER_MSGS::CARCMD"))
(defconstant user_msgs::CarCmd::*ARRIVE* 3)

(defun user_msgs::CarCmd-to-symbol (const)
  (cond
        ((= const 0) 'user_msgs::CarCmd::*HOLD*)
        ((= const 1) 'user_msgs::CarCmd::*REMOTE*)
        ((= const 2) 'user_msgs::CarCmd::*NAVIGATION*)
        ((= const 3) 'user_msgs::CarCmd::*ARRIVE*)
        (t nil)))

(defclass user_msgs::CarCmd
  :super ros::object
  :slots (_cmd ))

(defmethod user_msgs::CarCmd
  (:init
   (&key
    ((:cmd __cmd) 0)
    )
   (send-super :init)
   (setq _cmd (round __cmd))
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:serialization-length
   ()
   (+
    ;; uint8 _cmd
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _cmd
       (write-byte _cmd s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _cmd
     (setq _cmd (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get user_msgs::CarCmd :md5sum-) "1ce4d776e578234f635535a4932fa551")
(setf (get user_msgs::CarCmd :datatype-) "user_msgs/CarCmd")
(setf (get user_msgs::CarCmd :definition-)
      "uint8 cmd


# enum cmd
uint8 HOLD=0
uint8 REMOTE=1
uint8 NAVIGATION=2
uint8 ARRIVE=3


")



(provide :user_msgs/CarCmd "1ce4d776e578234f635535a4932fa551")


