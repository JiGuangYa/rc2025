#!/usr/bin/python3
# -*- coding: utf-8 -*-
import re
import ast
import cv2
import time
import copy
import numpy as np
from scipy.spatial.transform import Rotation
from utils import *
from arm_motions import *

import rospy
from std_msgs.msg import Bool, String, Int16 
from sensor_msgs.msg import CameraInfo, Image, JointState
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseStamped
from user_msgs.msg import CarCmd, ExitCondition, ObjDet, ObjDets


img_raw = None
car_cmd = CarCmd()
arrived = False
yolov5_dets = ObjDets()
left_arm_ik_servo_degree = JointState()
left_arm_get_cmd_ik = False
right_arm_ik_servo_degree = JointState()
right_arm_get_cmd_ik = False


def image_raw_cb(msg:Image):
    global img_raw
    bridge = CvBridge()
    img_raw = bridge.imgmsg_to_cv2(msg, 'bgr8')


def car_cmd_cb(msg:CarCmd):
    global car_cmd, arrived
    if not arrived and (car_cmd.cmd == CarCmd.NAVIGATION or car_cmd.cmd == CarCmd.HOLD) and msg.cmd == CarCmd.ARRIVE:
        arrived = True
    elif arrived and msg.cmd == CarCmd.NAVIGATION:
        arrived = False    
    car_cmd = msg


def yolov5_dets_cb(msg:ObjDets):
    global yolov5_dets
    yolov5_dets = msg


def left_arm_ik_servo_degree_cb(msg:JointState):
    global left_arm_ik_servo_degree, left_arm_get_cmd_ik
    left_arm_ik_servo_degree = msg
    left_arm_get_cmd_ik = True


def right_arm_ik_servo_degree_cb(msg:JointState):
    global right_arm_ik_servo_degree, right_arm_get_cmd_ik
    right_arm_ik_servo_degree = msg
    right_arm_get_cmd_ik = True


def main():
    global arrived, left_arm_get_cmd_ik, right_arm_get_cmd_ik

    rospy.init_node('mission_manage_node', anonymous=True)

    sort_basket_rois = rospy.get_param('/mission_magage/sort_basket_rois', [[(400, 130), (480, 170)],
                                                                            [(120, 140), (160, 200)],
                                                                            [(0, 200), (50, 320)],
                                                                            [(500, 260), (640, 380)]])
    
    go_to_grab_num = rospy.get_param('/mission_magage/go_to_grab/wp_num', 0)
    go_to_grab_wp = []
    for i in range(go_to_grab_num):
        go_to_grab_wp.append(rospy.get_param(f'/mission_magage/go_to_grab/wp_{i+1}'))

    transit_wp = rospy.get_param('/mission_magage/transit_pt/wp_1')

    grab_num = rospy.get_param('/mission_magage/grab/wp_num', 0)
    grab_wp = []
    for i in range(grab_num):
        grab_wp.append(rospy.get_param(f'/mission_magage/grab/wp_{i+1}'))
        
    put_num = rospy.get_param('/mission_magage/put/wp_num', 0)
    put_wp = []
    for i in range(put_num):
        put_wp.append(rospy.get_param(f'/mission_magage/put/wp_{i+1}'))

    go_back_wp = rospy.get_param('/mission_magage/go_back/wp_1')

    park_num = rospy.get_param('/mission_magage/park/wp_num', 0)
    park_wp = []
    for i in range(park_num):
        park_wp.append(rospy.get_param(f'/mission_magage/park/wp_{i+1}'))
    
    grab_param_l = GrabParam()
    grab_param_r = GrabParam()

    t_larm_2_car = rospy.get_param('/arm_left/translation', [0, 0, 0])
    t_rarm_2_car = rospy.get_param('/arm_right/translation', [0, 0, 0])
    t_cam_2_car = rospy.get_param('/ascamera/translation', [0, 0, 0])
    rpy_cam_2_car = rospy.get_param('/ascamera/rotation', [-90, 0, -90])
    sight_middle_h = rospy.get_param('/grab_param/sight_middle_h', 300)
    sight_middle_w = rospy.get_param('/grab_param/sight_middle_w', [269, 457])
    x_obj_to_cam_close = rospy.get_param('/grab_param/x_obj_to_cam_close', 0.205)
    x_obj_to_cam_middle = rospy.get_param('/grab_param/x_obj_to_cam_middle', 0.26)
    x_obj_to_cam_far = rospy.get_param('/grab_param/x_obj_to_cam_far', 0.34)
    x_j5_2_claw = rospy.get_param('/grab_param/x_j5_2_claw', 0.085)
    z_j5_2_claw = rospy.get_param('/grab_param/z_j5_2_claw', 0.0115)
    l_j5_2_claw = np.sqrt(x_j5_2_claw**2 + z_j5_2_claw**2)
    theta_j5_2_claw = np.rad2deg(np.arctan2(z_j5_2_claw, x_j5_2_claw))
    left_arm_front = rospy.get_param('turn_front', 90)
    left_arm_middle = rospy.get_param('turn_middle', 0)
    left_arm_back = rospy.get_param('turn_back', -90)
    left_arm_tight = rospy.get_param('/arm_left/claw_tight', 30)
    left_arm_loose = rospy.get_param('/arm_left/claw_loose', 60)
    right_arm_front = rospy.get_param('/arm_right/turn_front', 90)
    right_arm_middle = rospy.get_param('/arm_right/turn_middle', 0)
    right_arm_back = rospy.get_param('/arm_right/turn_back', -90)
    right_arm_tight = rospy.get_param('/arm_right/claw_tight', 30)
    right_arm_loose = rospy.get_param('/arm_right/claw_loose', 60)

    mission_time = rospy.get_param('/mission/mission_time', 300) - 25  # 留出返回时间

    image_raw_sub = rospy.Subscriber('/ascamera_hp60c/rgb0/image', Image, image_raw_cb)
    car_cmd_sub = rospy.Subscriber('/car_cmd', CarCmd, car_cmd_cb)
    yolov5_dets_sub = rospy.Subscriber('/ascamera_hp60c/rgb0/yolov5_detections', ObjDets, yolov5_dets_cb)
    left_arm_servo_degree_sub = rospy.Subscriber("/arm_left/arm_ik_servo_degree", JointState, left_arm_ik_servo_degree_cb)
    right_arm_servo_degree_sub = rospy.Subscriber("/arm_right/arm_ik_servo_degree", JointState, right_arm_ik_servo_degree_cb)

    ds_task_pub = rospy.Publisher('/deepseek/task', Bool, queue_size=1)
    ds_input_pub = rospy.Publisher('/deepseek/input_text', String, queue_size=1)
    at_target_pub = rospy.Publisher('/at_target_id', Int16, queue_size=1)
    car_target_pub = rospy.Publisher('/car_target', PoseStamped, queue_size=1)
    exit_condition_pub = rospy.Publisher('exit_condition', ExitCondition, queue_size=1)
    car_cmd_pub = rospy.Publisher('/car_cmd', CarCmd, queue_size=3)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
    left_arm_target_pose_pub = rospy.Publisher('/arm_left/arm_target', PoseStamped, queue_size=3)
    left_arm_servo_degree_pub = rospy.Publisher('/arm_left/arm_joint_states', JointState, queue_size=3)
    right_arm_target_pose_pub = rospy.Publisher('/arm_right/arm_target', PoseStamped, queue_size=3)
    right_arm_servo_degree_pub = rospy.Publisher('/arm_right/arm_joint_states', JointState, queue_size=3)

    # ################################################################################
    # 输入任务指令 等待deepseek输出
    ds_input = String()
    input_temp = input("输入任务指令\n>>>")
    ds_input.data = "在此填入自定义附加提示词" \
                    + input_temp + "。"
    ds_input_pub.publish(ds_input)
    # ################################################################################

    t0 = time.time()
    rospy.loginfo("任务开始\n")

    # ################################################################################
    # 等待deepseek处理得到的结果
    ds_output: String = rospy.wait_for_message('/deepseek/output_text', String, timeout=None)
    # ################################################################################

    # # ################################################################################
    # # 手动指定结果 仅供调试使用
    # ds_output = String()
    # ds_output.data = "[[熟柠檬, 绿色], [坏柠檬, 蓝色], [坏樱桃, 红色], [熟樱桃, 黄色], [2]]"
    # # ################################################################################

    # 处理deepseek的输出，将字符串转换为列表
    orig_str = ds_output.data
    if "色" in orig_str:
        orig_str = orig_str.replace("色", "")
    try:
        mission = ast.literal_eval(orig_str)
        print(mission)
    except ValueError:
        processed = re.sub(r'(?<=[\[,\s])([\u4e00-\u9fa5]+)(?=[,\]\s])', r'"\1"', orig_str)
        mission = ast.literal_eval(processed)
        print(mission)
    # 将列表中的任务转换为代号，便于后续任务执行
    grab_mission = mission[:4]
    park_mission = mission[4][0]
    for item in grab_mission:
        if isinstance(item, list) and len(item) == 2:
            if item[0] in fruit_dict:
                item[0] = fruit_dict[item[0]]
            if item[1] in color_dict:
                item[1] = color_dict[item[1]]
    fruit_cmd = [grab_mission[i][0] for i in range(4)]
    basket_color_cmd = [grab_mission[i][1] for i in range(4)]
    print("抓取放置任务配对: ", grab_mission)  # [[4, 3], [0, 2], [1, 0], [5, 1]]
    print("泊车任务: ", park_mission)  # 2
    
    # 结束deepseek节点 释放内存
    ds_task_pub.publish(Bool(False))

    camera_info_msg: CameraInfo = rospy.wait_for_message('/ascamera_hp60c/rgb0/camera_info', CameraInfo, timeout=None)
    grab_param_l.cx = camera_info_msg.K[2]
    grab_param_l.cy = camera_info_msg.K[5]
    grab_param_l.fx = camera_info_msg.K[0]
    grab_param_l.fy = camera_info_msg.K[4]
    grab_param_l.t_arm_2_car = t_larm_2_car
    grab_param_l.t_cam_2_car = t_cam_2_car
    grab_param_l.rpy_cam_2_car = rpy_cam_2_car
    grab_param_l.sight_middle_h = sight_middle_h
    grab_param_l.sight_middle_w = sight_middle_w
    grab_param_l.x_obj_to_cam_close = x_obj_to_cam_close
    grab_param_l.x_obj_to_cam_middle = x_obj_to_cam_middle
    grab_param_l.x_obj_to_cam_far = x_obj_to_cam_far
    grab_param_l.l_j5_2_claw = l_j5_2_claw
    grab_param_l.theta_j5_2_claw = theta_j5_2_claw
    grab_param_r.cx = camera_info_msg.K[2]
    grab_param_r.cy = camera_info_msg.K[5]
    grab_param_r.fx = camera_info_msg.K[0]
    grab_param_r.fy = camera_info_msg.K[4]
    grab_param_r.t_arm_2_car = t_rarm_2_car
    grab_param_r.t_cam_2_car = t_cam_2_car
    grab_param_r.rpy_cam_2_car = rpy_cam_2_car
    grab_param_r.sight_middle_h = sight_middle_h
    grab_param_r.sight_middle_w = sight_middle_w
    grab_param_r.x_obj_to_cam_close = x_obj_to_cam_close
    grab_param_r.x_obj_to_cam_middle = x_obj_to_cam_middle
    grab_param_r.x_obj_to_cam_far = x_obj_to_cam_far
    grab_param_r.l_j5_2_claw = l_j5_2_claw
    grab_param_r.theta_j5_2_claw = theta_j5_2_claw
    arm_servo_degree = JointState()
    arm_servo_degree.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    # 出发前订阅相机画面，检测放置框的顺序
    r = rospy.Rate(30)
    seq_lists = []
    while not rospy.is_shutdown():
        if img_raw is not None:
            basket_color_seq_ = sort_basket_color_seq(img_raw, sort_basket_rois)
            seq_lists.append(basket_color_seq_)
            if len(seq_lists) == 10:
                break
        r.sleep()
    cv2.destroyAllWindows()
    basket_color_seq = find_most_seq(seq_lists)
    print("放置区顺序: ", basket_color_seq)

    chassis_turn(180, car_cmd_pub, cmd_vel_pub)

    at_target = Int16()
    car_target = PoseStamped()
    exit_condition = ExitCondition()

    print("\n离开起始区...\n")
    for i in range(go_to_grab_num):
        arrived = False
        pub_wp(go_to_grab_wp[i], 
               car_cmd, at_target, car_target, exit_condition,
               car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub)
        while not rospy.is_shutdown():
            if not arrived:
                time.sleep(1)
            else:
                break

    chassis_turn(180, car_cmd_pub, cmd_vel_pub)

    print("\n前往中间点...\n")
    arrived = False
    pub_wp(transit_wp, 
           car_cmd, at_target, car_target, exit_condition,
           car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub)
    while not rospy.is_shutdown():
        if not arrived:
            time.sleep(1)
        else:
            break
    
    # 采摘/运输
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    grab_wp_cnt = 0

    while not rospy.is_shutdown() and (time.time() - t0 < mission_time) and grab_wp_cnt < grab_num:
        # 前往采摘位置
        print(f"\n前往{grab_wp_cnt+1}号采摘位置...\n")
        arrived = False
        pub_wp(grab_wp[grab_wp_cnt], 
               car_cmd, at_target, car_target, exit_condition,
               car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub)
        while not rospy.is_shutdown():
            if not arrived:
                time.sleep(1)
            else:
                break

        # 采摘
        print(f"\n到达 {grab_wp_cnt+1}号 采摘位置 开始检测\n")
        t1 = time.time()
        yolov5_dets_ = None
        while not rospy.is_shutdown() and yolov5_dets_ is None:
            if time.time() - t1 > 2:
                rospy.logwarn("超时")
                break
            if not yolov5_dets.num:
                rospy.Rate(1).sleep()
                continue
            yolov5_dets_ = copy.copy(yolov5_dets)

        if yolov5_dets_ is None:
            print(f"\n{grab_wp_cnt+1}号 采摘位置已采摘完成\n")
            grab_wp_cnt += 1
            continue

        print("\n检测到水果 准备抓取\n")
        assign_left = False
        assign_right = False
        fruit_in_arm = [-1, -1]  # [右臂抓取结果, 左臂抓取结果]
        for det in yolov5_dets_.dets:
            det: ObjDet
            if det.class_id not in fruit_cmd:
                continue
            
            target_center_pix = [det.bbox.x, det.bbox.y]
            
            if target_center_pix[0] < sight_middle_w[1] and not assign_left:
                # 左臂
                assign_left = True
                pose = calc_j5_from_claw(True, target_center_pix, 0, grab_param_l)
                if pose is None:
                    continue
                pose_in_left_arm_base = calc_grab("/arm_left/", pose, tf_buffer)
                left_arm_target_pose_pub.publish(pose_in_left_arm_base)
                t2 = time.time()
                while not left_arm_get_cmd_ik and (time.time() - t2 < 2):
                    pass
                if left_arm_get_cmd_ik:
                    grab_degree = left_arm_ik_servo_degree.position
                    left_arm_get_cmd_ik = False
                    grab(grab_degree, left_arm_loose, left_arm_tight, True, arm_servo_degree, left_arm_servo_degree_pub)
                    fruit_in_arm[1] = det.class_id
            
            elif target_center_pix[0] >= sight_middle_w[1] and not assign_right:
                # 右臂
                assign_right = True
                pose = calc_j5_from_claw(False, target_center_pix, 0, grab_param_r)
                if pose is None:
                    continue
                pose_in_right_arm_base = calc_grab("/arm_right/", pose, tf_buffer)
                right_arm_target_pose_pub.publish(pose_in_right_arm_base)
                t2 = time.time()
                while not right_arm_get_cmd_ik and (time.time() - t2 < 2):
                    pass
                if right_arm_get_cmd_ik:
                    grab_degree = right_arm_ik_servo_degree.position
                    right_arm_get_cmd_ik = False
                    grab(grab_degree, right_arm_loose, right_arm_tight, False, arm_servo_degree, right_arm_servo_degree_pub)
                    fruit_in_arm[0] = det.class_id

            if assign_left and assign_right:
                # 左右臂都有抓取
                break
                
        if fruit_in_arm == [-1, -1]:
            print(f"\n{grab_wp_cnt+1}号 采摘位置已无可采摘水果\n")
            grab_wp_cnt += 1
            continue

        print("\n前往中间点...\n")
        arrived = False
        pub_wp(transit_wp, 
               car_cmd, at_target, car_target, exit_condition,
               car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub)
        while not rospy.is_shutdown():
            if not arrived:
                time.sleep(1)
            else:
                break

        for index, fruit in enumerate(fruit_in_arm):
            if fruit == -1:
                print(f"\n{'左' if index else '右'}臂里无水果\n")
                continue

            basket_color = basket_color_cmd[fruit_cmd.index(fruit)]
            put_basket = basket_color_seq.index(basket_color)

            print(f"\n前往 {put_basket+1} 号放置位...\n")

            if put_basket in [1, 2]:
                chassis_turn(90, car_cmd_pub, cmd_vel_pub)

            arrived = False
            pub_wp(put_wp[put_basket], 
                   car_cmd, at_target, car_target, exit_condition,
                   car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub)
            while not rospy.is_shutdown():
                if not arrived:
                    time.sleep(1)
                else:
                    break
            
            if put_basket == 3:
                chassis_turn(90, car_cmd_pub, cmd_vel_pub)

            print(f"{'左' if index else '右'}臂 将 {fruit_dict_.get(fruit)} 放置到 {color_dict_.get(basket_color)}框")
            if index:
                put(left_arm_front, left_arm_loose, left_arm_tight, True, arm_servo_degree, left_arm_servo_degree_pub)
            else:
                put(right_arm_front, right_arm_loose, right_arm_tight, True, arm_servo_degree, right_arm_servo_degree_pub)

            if put_basket in [1, 2]:
                chassis_turn(-90, car_cmd_pub, cmd_vel_pub)
            elif put_basket == 3:
                chassis_turn(-180, car_cmd_pub, cmd_vel_pub)

            print("\n前往中间点...\n")
            arrived = False
            pub_wp(transit_wp, 
                car_cmd, at_target, car_target, exit_condition,
                car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub)
            while not rospy.is_shutdown():
                if not arrived:
                    time.sleep(1)
                else:
                    break


    print("\n前往中间点...\n")
    arrived = False
    pub_wp(transit_wp, 
        car_cmd, at_target, car_target, exit_condition,
        car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub)
    while not rospy.is_shutdown():
        if not arrived:
            time.sleep(1)
        else:
            break

    print("\n返回\n")
    arrived = False
    pub_wp(go_back_wp, 
           car_cmd, at_target, car_target, exit_condition,
           car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub)
    while not rospy.is_shutdown():
        if not arrived:
            time.sleep(1)
        else:
            break

    chassis_turn(180, car_cmd_pub, cmd_vel_pub)
    
    print(f"\n停入 {park_mission}号 车位\n")
    arrived = False
    pub_wp(park_wp[park_mission-1], 
           car_cmd, at_target, car_target, exit_condition,
           car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub)
    while not rospy.is_shutdown():
        if not arrived:
            time.sleep(1)
        else:
            break
    

if __name__ == "__main__":
    main()
