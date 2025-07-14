
(cl:in-package :asdf)

(defsystem "user_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CannonStatus" :depends-on ("_package_CannonStatus"))
    (:file "_package_CannonStatus" :depends-on ("_package"))
    (:file "CarCmd" :depends-on ("_package_CarCmd"))
    (:file "_package_CarCmd" :depends-on ("_package"))
    (:file "ExitCondition" :depends-on ("_package_ExitCondition"))
    (:file "_package_ExitCondition" :depends-on ("_package"))
    (:file "GimbalPoseDeg" :depends-on ("_package_GimbalPoseDeg"))
    (:file "_package_GimbalPoseDeg" :depends-on ("_package"))
    (:file "ObjBbox" :depends-on ("_package_ObjBbox"))
    (:file "_package_ObjBbox" :depends-on ("_package"))
    (:file "ObjDet" :depends-on ("_package_ObjDet"))
    (:file "_package_ObjDet" :depends-on ("_package"))
    (:file "ObjDets" :depends-on ("_package_ObjDets"))
    (:file "_package_ObjDets" :depends-on ("_package"))
    (:file "PID" :depends-on ("_package_PID"))
    (:file "_package_PID" :depends-on ("_package"))
  ))