#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64

def angle_callback(msg):
    """
    接收到舵机角度指令时的回调函数。
    """
    # 获取角度值 (单位: 度)
    angle_degrees = msg.data
    
    rospy.loginfo("接收到舵机角度指令: {:.2f} 度".format(angle_degrees))
    
    # --- 在这里添加您的硬件控制逻辑 ---
    # 例如，将角度值转换为PWM信号并发送给舵机控制器
    # pwm_value = convert_angle_to_pwm(angle_degrees)
    # send_to_servo_controller(pwm_value)
    # ----------------------------------

def servo_controller_node():
    """
    初始化ROS节点并订阅话题。
    """
    # 初始化节点
    rospy.init_node('servo_controller', anonymous=True)
    
    # 订阅舵机角度指令话题
    rospy.Subscriber("/servo_angle_command", Float64, angle_callback)
    
    rospy.loginfo("舵机控制节点已启动，等待角度指令...")
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_controller_node()
    except rospy.ROSInterruptException:
        pass
