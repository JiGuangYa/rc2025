# 智慧果园机器人项目

本项目是为“智慧果园”机器人竞赛设计的 ROS (Robot Operating System) 上位机软件。项目包含两个独立的 Catkin 工作空间：`catkin_ws` 和 `mission_ws`，分别负责机器人的核心功能和高级任务逻辑。

## 项目结构

整个项目由两个核心部分组成：

### 1. `catkin_ws` (核心功能工作空间)

此工作空间包含了机器人的基础驱动、传感器接口、核心算法和通用消息定义。

- **`src/actuator`**: 控制机器人硬件执行器，如机械臂、底盘、云台等。
- **`src/algorithm`**: 包含机器人自主运行所需的核心算法，例如：
    - **`detection`**: 目标检测算法。
    - **`llm`**: 大语言模型接口。
    - **`navigation`**: 导航与路径规划。
    - **`slam`**: 即时定位与地图构建。
- **`src/sensor`**: 提供对各类传感器的支持，例如：
    - **`ascamera`**: AS Camera 深度相机。
    - **`livox`**: Livox 激光雷达。
    - **`realsense`**: RealSense 深度相机。
    - **`rplidar_ros`**: RPLIDAR 激光雷达。
    - **`usb_cam`**: USB 摄像头。
- **`src/user_msgs`**: 定义了项目专用的 ROS 消息类型，用于各节点间的数据交换。
- **`src/user_params`**: 管理和加载机器人运行所需的各类参数。
- **`src/vision_opencv`**: 封装了基于 OpenCV 的计算机视觉功能。

### 2. `mission_ws` (任务逻辑工作空间)

此工作空间专注于实现比赛的具体任务流程和高层决策。

- **`src/mission`**: 包含比赛任务的逻辑控制节点，负责协调 `catkin_ws` 中的各个功能模块以完成复杂的比赛任务。

## 环境要求

- **操作系统**: Ubuntu 18.04 LTS 或更高版本
- **ROS 版本**: ROS Melodic Morenia 或更高版本
- **依赖库**:
    - `catkin`
    - `OpenCV`
    - `PCL` (Point Cloud Library)
    - 其他在各功能包 `package.xml` 中声明的依赖项

## 编译与运行

### 1. 编译工作空间

请按顺序编译两个工作空间。首先编译 `catkin_ws`，因为它包含了 `mission_ws` 所需的基础消息和库。

**重要提示**: 本仓库使用 `.gitignore` 文件忽略了 `build/` 和 `devel/` 目录。在克隆项目后，您需要首先在本地编译工作空间，以生成这些必要的目录，然后才能设置环境和运行节点。

**编译 `catkin_ws`:**
```bash
cd /path/to/your/project/catkin_ws
catkin_make
```

**编译 `mission_ws`:**
```bash
cd /path/to/your/project/mission_ws
catkin_make
```

### 2. 设置环境

在**新的终端**中，分别 source 两个工作空间的 `setup.bash` 文件，以便 ROS 环境能够找到对应的功能包。

```bash
# 在第一个终端中设置 catkin_ws
source /path/to/your/project/catkin_ws/devel/setup.bash

# 在第二个终端中设置 mission_ws
source /path/to/your/project/mission_ws/devel/setup.bash
```

**注意**: 为了方便，您可以将以上 `source` 命令添加到您的 `~/.bashrc` 文件中。

### 3. 运行节点

根据比赛需求，在已设置好环境的终端中，使用 `roslaunch` 或 `rosrun` 命令启动相应的节点。

例如，启动一个核心功能节点：
```bash
# 确保已 source catkin_ws
roslaunch some_package_in_catkin_ws some_launch_file.launch
```

启动任务逻辑节点：
```bash
# 确保已 source mission_ws
roslaunch mission mission_control.launch
```

## 维护与开发

- **代码风格**: 请遵循 ROS C++ 和 Python 的标准编码规范。
- **新功能**:
    - 如果是底层驱动、算法或通用工具，请在 `catkin_ws` 中创建新的功能包。
    - 如果是与特定任务相关的高层逻辑，请在 `mission_ws` 的 `mission` 包内进行开发。
- **Git**: 在提交代码前，请确保代码能够成功编译并通过基本测试。

## 新增模块：水果抓取任务 (`fruit_picking_mission`)

为了完成“抓取树上果子并分类放入背后桶里”的任务，我们在 `mission_ws/src/mission` 中添加了 `fruit_picking_mission` 包。该包是此项任务的核心控制器。

### 1. 核心节点: `mission_manager.py`

这是任务的总指挥节点，其主要职责是：

- **接收任务**: 通过一个名为 `pick_fruit` 的 Action Server 接收抓取指令。指令中包含要抓取的水果类型（如 `apple`, `banana`）。
- **感知水果**: 订阅 `/obj_dets` 话题 (类型: `user_msgs/ObjDets`)，获取由 `yolov5_rknn` 等视觉节点发布的水果位置信息。
- **协调硬件**: 调用机械臂和舵机的服务来执行物理操作。

### 2. 接口定义: `PickFruit.action`

我们定义了一个 Action (`fruit_picking_mission/action/PickFruit.action`) 来管理抓取流程，它包含：
- **Goal**: `string fruit_type` - 要抓取的水果名称。
- **Result**: `bool success` - 任务是否成功。
- **Feedback**: `string status` - 任务执行过程中的实时状态。

### 3. 舵机控制与配置

舵机用于旋转水果桶，确保不同种类的水果放入对应的格子。控制逻辑位于 `mission_manager.py` 中。

- **配置文件**: 在 `mission_manager.py` 脚本内部，有一个名为 `bucket_positions` 的字典，它定义了**水果类型**到**舵机旋转角度**的映射关系。

  ```python
  self.bucket_positions = {
      'apple': 0,      # 苹果 -> 0度
      'banana': 90,   # 香蕉 -> 90度
      'orange': 180,  # 橙子 -> 180度
      'grape': 270    # 葡萄 -> 270度
  }
  ```

- **控制流程**: 当 `mission_manager` 接收到抓取某种水果的任务时，在机械臂将水果抓取到桶上方后，它会查询此字典，并调用舵机控制服务，将桶旋转到正确的角度，然后释放水果。

- **待办事项**: 舵机控制目前是**伪代码**。开发者需要：
  1. 在 `catkin_ws/src/actuator/servo_control` 中实现一个ROS服务，该服务可以接收一个角度值并控制舵机转动。
  2. 在 `mission_manager.py` 中，取消相关代码的注释，并调用该服务。

### 4. 如何运行

1.  **编译工作空间**: 确保 `mission_ws` 已被成功编译 (`catkin_make`)。
2.  **启动任务管理器**:
    ```bash
    # 确保已 source mission_ws/devel/setup.bash
    rosrun fruit_picking_mission mission_manager.py
    ```
3.  **发送测试指令**:
    ```bash
    # 在新终端中
    rosrun actionlib axclient.py /pick_fruit
    ```
    在弹出的窗口中，输入目标，例如 `fruit_type: "apple"`，然后点击 “Send Goal”。

---
*该文档由 Gemini 自动生成和更新。*
