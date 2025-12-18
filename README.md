# 🦾 Vision-Guided Top-Down Pick & Place Using YOLO + MoveIt 2

### **ROS 2 Humble • MoveIt 2 • YOLOv8 • Panda Robot Simulation**

<div align="center">

![Panda Robot]((https://robodk.com/robot/Franka/Emika-Panda))

<img width="553" height="572" alt="Screenshot from 2025-12-18 15-46-36" src="https://github.com/user-attachments/assets/619c58f2-60c0-49f5-a622-bd6f8595fabb" />


[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)]()
[![MoveIt](https://img.shields.io/badge/MoveIt2-Motion%20Planning-purple.svg)]()
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Object%20Detection-green.svg)]()
[![Python](https://img.shields.io/badge/Python-3.10-blue.svg)]()

</div>

---

# 📌 **Project Overview**

This project implements a **vision-guided robotic pick-and-place pipeline** in simulation using:

* **Franka Panda Robot (MoveIt + RViz)**
* **YOLOv8 Object Detection**
* **Collision Objects (Shelf, Apple, Bin)**
* **IK solving & Trajectory Planning using MoveIt**
* **Top-down grasp strategy**
* **Gripper control via ros2_control**
* **Built-in performance metrics (IK time, planning time, total cycle time)**

The robot detects a target object (e.g., an apple), moves directly above it, lowers vertically, grasps it, lifts, moves to a bin, and drops it — **all autonomously**.

---

# 🧩 **Key Features**

### 🔍 **1. YOLO-based vision perception**

* YOLOv8 reads a static image (`item.jpg`)
* Detects the object class and publishes `/detected_object_pose`

### 🧠 **2. MoveIt-based IK + Planning**

* Computes IK using `/compute_ik`
* Plans motions using `/plan_kinematic_path`
* Executes trajectories using `/execute_trajectory`

### 🤖 **3. Top-down grasp strategy**

Ensures stable grasping by orienting the gripper vertically:

```python
roll = -π/2, pitch = 0, yaw = 0
```

### 📦 **4. Custom planning scene**

Environment injected into MoveIt via ROS 2 topic:

* A **shelf**
* An **apple**
* A **collection bin**

### ⏱️ **5. Built-in performance metrics**

* IK solve time
* Planning time
* Execution time
* Full pipeline time

Printed each run for benchmarking.

---

# 📸 **System Architecture**

```
+------------------+       +----------------------+       +--------------------------+
| YOLO Detector    | ---> | Pick & Place Node    | --->  | MoveIt 2 Planning         |
| (yolo_node.py)   |      | (IK, Planning, Exec) |       | (IK, OMPL, Controller)    |
+------------------+       +----------------------+       +--------------------------+
            ↑                                                    |
            |                                                    |
            +----------------------------------------------------+
                        Planning Scene Node (Shelf, Apple, Bin)
```

---

# 📂 **Workspace Structure**

```
panda_ws/
│── src/
│    ├── panda_scene/          # Planning scene publisher
│    ├── panda_perception/     # YOLOv8 detector
│    └── panda_manipulation/   # Pick-and-place controller
│
├── install/                   # (generated)
├── build/                     # (generated)
└── log/                       # (generated)
```

---

# ⚙️ **Installation Instructions (From Zero)**

### **1️⃣ Install ROS 2 Humble + MoveIt**

```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-moveit
sudo apt install ros-humble-control-msgs ros-humble-controller-manager
```

---

### **2️⃣ Setup workspace**

```bash
mkdir -p ~/panda_ws/src
cd ~/panda_ws
```

Clone or copy these packages into `src/`:

```
panda_scene/
panda_perception/
panda_manipulation/
```

---

### **3️⃣ Install YOLO dependencies**

```bash
pip install ultralytics opencv-python numpy
```

---

### **4️⃣ Build Workspace**

```bash
cd ~/panda_ws
colcon build --symlink-install
source install/setup.bash
```

---

# ▶️ **Usage Instructions**

## **Terminal 1 — Start MoveIt + RViz**

```bash
source /opt/ros/humble/setup.bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

---

## **Terminal 2 — Start Planning Scene**

```bash
source ~/panda_ws/install/setup.bash
ros2 run panda_scene setup_scene
```

---

## **Terminal 3 — Run YOLO Detector**

```bash
ros2 run panda_perception yolo_node --ros-args -p image_path:=/home/$USER/item.jpg
```

Ensure `item.jpg` exists.

---

## **Terminal 4 — Run Pick and Place**

```bash
ros2 run panda_manipulation pick_place
```

You will observe:

* Hover → descend → close gripper → lift → move to bin → open gripper
* Metrics printed automatically

---

# 📏 **Performance Metrics Logged**

Example terminal output:

```
IK solve time: 0.0027 sec
Planning time: 0.281 sec
Execution time: 1.33 sec
TOTAL pipeline time: 8.11 sec
```

Metrics are computed inside:

```
panda_manipulation/pick_place.py
```

You may graph these later (MatPlotLib, Excel, etc.)

---

# 🔧 **Technologies Used**

| Component             | Purpose                                 |
| --------------------- | --------------------------------------- |
| **ROS 2 Humble**      | Node communication, actions, services   |
| **MoveIt 2**          | IK, motion planning, collision checking |
| **YOLOv8**            | Object detection                        |
| **RViz2**             | Visualization                           |
| **ros2_control**      | Executing joint/trajectory commands     |
| **OMPL / RRTConnect** | Motion planning algorithm               |
| **Python 3.10**       | Implementation of all custom logic      |

---

# 🚀 **Future Extensions**

* Real-time camera stream instead of static image
* Multi-object pick & place
* Dynamic grasp orientation based on detection mask
* Add grasp quality metrics
* Use a reinforcement learning-based placer
* Extend to real Panda robot or OpenManipulator-X

---

# 📜 **License**

MIT License — modify and reuse freely.

---

# 🙌 **Contributors**

| Name             | Role                                           |
| ---------------- | ---------------------------------------------- |
| **Your Name**    | Vision, perception pipeline, ROS architecture  |
| **Partner Name** | Motion planning, control, trajectory execution |

---

# ⭐ **How to Support**

If this project helped you, please ⭐ star the repository!

---


