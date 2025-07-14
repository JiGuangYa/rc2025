#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from fruit_picking_mission.msg import PickFruitAction
from user_msgs.msg import ObjDets
from servo_control.srv import SetAngle

# 假设的服务，您需要实现它们
# from some_arm_pkg.srv import MoveArm, Gripper

class MissionManager:
    def __init__(self):
        rospy.init_node('mission_manager')

        # 水果 -> 舵机角度 (需要您来填充)
        self.bucket_positions = {
            'apple': 0,
            'banana': 90,
            'orange': 180,
            'grape': 270
        }

        # 订阅水果检测结果
        self.detected_fruits = {}
        rospy.Subscriber('/obj_dets', ObjDets, self.detection_callback)

        # 创建Action Server
        self._as = actionlib.SimpleActionServer('pick_fruit', PickFruitAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # 创建服务客户端
        rospy.loginfo("Waiting for servo service...")
        rospy.wait_for_service('/set_servo_angle')
        self.set_servo_angle_client = rospy.ServiceProxy('/set_servo_angle', SetAngle)
        rospy.loginfo("Servo service connected.")

        rospy.loginfo("Mission Manager is ready.")

    def detection_callback(self, data):
        # 更新检测到的水果列表和它们的位置
        self.detected_fruits = {det.label: det for det in data.dets}

    def execute_cb(self, goal):
        rospy.loginfo(f"Received goal to pick: {goal.fruit_type}")

        # 1. 检查水果是否存在
        if goal.fruit_type not in self.detected_fruits:
            rospy.logwarn(f"Fruit '{goal.fruit_type}' not detected.")
            self._as.set_aborted(text=f"Fruit '{goal.fruit_type}' not detected.")
            return

        fruit_pose = self.detected_fruits[goal.fruit_type].bbox

        # 2. 移动机械臂到水果位置 (伪代码)
        rospy.loginfo(f"Moving arm to pick {goal.fruit_type} at {fruit_pose}")
        # move_arm_client(fruit_pose)
        # gripper_client('close')

        # 3. 移动机械臂到桶上方 (伪代码)
        rospy.loginfo("Moving arm to bucket")
        # move_arm_client('bucket_drop_pose')

        # 4. 旋转舵机到正确的位置
        if goal.fruit_type in self.bucket_positions:
            angle = self.bucket_positions[goal.fruit_type]
            rospy.loginfo(f"Rotating bucket to {angle} degrees for {goal.fruit_type}")
            try:
                response = self.set_servo_angle_client(angle)
                if not response.success:
                    rospy.logerr("Failed to set servo angle.")
                    self._as.set_aborted(text="Failed to control servo.")
                    return
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                self._as.set_aborted(text="Failed to connect to servo service.")
                return
        else:
            rospy.logwarn(f"No bucket position defined for {goal.fruit_type}")

        # 5. 释放水果 (伪代码)
        rospy.loginfo(f"Dropping {goal.fruit_type}")
        # gripper_client('open')

        # 6. 返回初始位置 (伪代码)
        rospy.loginfo("Returning arm to home position")
        # move_arm_client('home_pose')

        # 7. 设置Action结果
        result = self._as.get_default_result()
        result.success = True
        result.message = f"Successfully picked and placed {goal.fruit_type}."
        self._as.set_succeeded(result)

if __name__ == '__main__':
    try:
        MissionManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
