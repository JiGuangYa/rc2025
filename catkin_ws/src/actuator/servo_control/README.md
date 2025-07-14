# 舵机控制节点 (Servo Control Node)

这是一个简单的 ROS 节点，用于演示如何控制一个舵机。它作为一个基础模板，方便您在此之上进行二次开发。

## 功能

- 订阅 `/servo_angle_command` 话题，消息类型为 `std_msgs/Float64`。
- 接收到的 `data` 字段被视为舵机的目标角度（单位：度）。
- 接收到指令后，节点会在日志中打印出目标角度。

## 如何运行

1.  **编译工作空间**:
    确保您已经在 `catkin_ws` 的根目录下执行了 `catkin_make`。
    ```bash
    cd /path/to/your/catkin_ws
    catkin_make
    ```

2.  **设置环境**:
    在终端中 source 工作空间的环境。
    ```bash
    source /path/to/your/catkin_ws/devel/setup.bash
    ```

3.  **运行节点**:
    ```bash
    rosrun servo_control servo_node.py
    ```

## 如何测试

打开一个新的终端，并执行以下命令来发布一个角度指令（例如，90度）：

```bash
rostopic pub /servo_angle_command std_msgs/Float64 "data: 90.0"
```

您应该能在运行节点的终端中看到相应的日志输出。

## 如何开发

您可以在 `scripts/servo_node.py` 文件中标记的位置添加与真实硬件交互的代码，例如通过串口或 GPIO 发送 PWM 信号。
