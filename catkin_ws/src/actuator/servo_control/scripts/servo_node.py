#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from servo_control.srv import SetAngle, SetAngleResponse

def handle_set_angle(req):
    """
    处理设置舵机角度的服务请求。
    """
    angle_degrees = req.angle
    rospy.loginfo(f"接收到舵机角度指令: {angle_degrees:.2f} 度")

    # --- 在这里添加您的硬件控制逻辑 ---
    # 例如，将角度值转换为PWM信号并发送给舵机控制器
    # success = send_to_servo_controller(angle_degrees)
    # ----------------------------------
    
    # 模拟一个成功的操作
    success = True 

    if success:
        rospy.loginfo("舵机角度设置成功。")
        return SetAngleResponse(success=True, message="Angle set successfully.")
    else:
        rospy.logerr("舵机角度设置失败。")
        return SetAngleResponse(success=False, message="Failed to set angle.")

def servo_service_node():
    """
    初始化ROS节点并启动服务。
    """
    rospy.init_node('servo_service_node')
    
    # 创建一个名为 /set_servo_angle 的服务
    s = rospy.Service('/set_servo_angle', SetAngle, handle_set_angle)
    
    rospy.loginfo("舵机控制服务已就绪，等待请求...")
    rospy.spin()

if __name__ == "__main__":
    try:
        servo_service_node()
    except rospy.ROSInterruptException:
        pass