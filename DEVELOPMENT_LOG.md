# Development Log

This file summarizes the development progress of the project. It is used to provide context across different sessions.

---

## 2025-07-14: Implement Fruit Picking Mission

- **Objective**: Create the core logic for a fruit picking and sorting task.

- **Actions Taken**:
  - Created a new ROS package `fruit_picking_mission` in `mission_ws/src/mission/`.
  - Defined a ROS Action `PickFruit.action` to handle the asynchronous picking task. The action goal is the `fruit_type` (string) and it returns a `success` boolean.
  - Implemented the main control node `mission_manager.py` inside the new package. This script includes:
    - An Action Server to receive picking goals.
    - A subscriber for fruit detection messages (`/obj_dets`).
    - Placeholder logic for calling arm and servo services.
    - A dictionary `bucket_positions` to map fruit types to the servo's rotation angle for sorting.
  - Updated the main `README.md` file to include detailed documentation about the new `fruit_picking_mission` package, its architecture, and how to run it.

---

## 2025-07-14: Refactor Servo Control to a ROS Service

- **Objective**: Improve the robustness of servo control by changing from a Topic-based to a Service-based architecture.

- **Actions Taken**:
  - **Created `SetAngle.srv`**: Defined a new service in `catkin_ws/src/actuator/servo_control/srv/`. The service takes a float `angle` and returns a boolean `success`.
  - **Updated `servo_node.py`**: Modified the script to act as a ROS Service Server for `/set_servo_angle`. It no longer subscribes to a topic.
  - **Updated `mission_manager.py`**: Replaced the placeholder comment with a ROS Service Client that calls `/set_servo_angle`. This makes the action of rotating the bucket a blocking call, ensuring the rotation is complete before the arm drops the fruit.
  - **Updated Build Files**: Modified `CMakeLists.txt` and `package.xml` in `servo_control` to support service generation.
