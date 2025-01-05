# **Homework 1** 🎓🤖

Welcome to Homework 2! The primary goal of this homework is to control a robotic arm to follow pre-defined trajectories with smooth and accurate movements. The code will:
- Replace the existing velocity profile with a cubic polynomial, providing a smoother transition between positions.
- Create both **linear** and **circular** trajectories for the robot.
- Tune the control gains to ensure the manipulator follows the desired trajectory accurately.
- Implement an inverse dynamics controller that works in operational space, improving the robot's performance in tasks that require precision in the Cartesian space.

By running this code, the robotic manipulator will be able to follow trajectories while ensuring that torques and forces are smoothly applied, leading to realistic motion in the **Gazebo** simulation.

### **Choosing the Trajectory Type** 🎯

In the code, the value of the parameter `k` determines which trajectory the robot will execute. The possible values of `k` correspond to different types of trajectories with varying velocity profiles. By changing the value of `k`, you can choose between **circular** or **linear** trajectories, and between using a **cubic polynomial** or a **trapezoidal velocity profile**. Here's a quick guide to the available options:  

| **Value of `k`** | **Trajectory Type**       | **Velocity Profile**         |
|------------------|---------------------------|------------------------------|
| `1`              | Circular                  | Cubic Polynomial             |
| `2`              | Circular                  | Trapezoidal Velocity Profile |
| `3`              | Linear                    | Cubic Polynomial             |
| `4`              | Linear                    | Trapezoidal Velocity Profile |

---

### **Usage** 🛠️

---

####  **Position Controller** 🧭

1. Open the first terminal and run:
    ```bash
    ros2 launch iiwa_bringup iiwa.launch.py
    ```
2. In a second terminal, run:
    ```bash
    ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p k:=1
    ```
> **Note:**  
> The robot will be launched by default with the **position interface**.  
---

#### **Velocity Controller** ⚡

1. Open the first terminal and run:
    ```bash
    ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
    ```
2. In a second terminal, run:
    ```bash
    ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity -p k:=1
    ```
> **Note:**  
> The robot will be launched by with the **velocity interface**.  
---

#### **Effort Controller** 💪

1. Open the first terminal and run:
    ```bash
    ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller" use_sim:=true
    ```
2. Wait for 5 seconds and press the **Play button** in **Gazebo**. ⏱️🎮
3. In a second terminal, run:
    ```bash
    ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=effort -p k:=1
    ```

> **Note:**  
> The robot will be launched by default with the **effort interface** in the **joint space**.  
> If you want to use the **effort controller** in the **operational space**, run: 
    ```bash
    ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=effort -p k:=1 -p effort:=cartesian
    ```

---

#### **Plotting Torque in MATLAB** 📊

To plot the **torque** in **MATLAB**, modify the script by inserting the **path** of the folder containing the **bag file**.

**To generate the bag file:**
	```bash
	ros2 bag record -o subset /effort_controller/commands
	```
