## Agri Autonomy ROS 2

ROS 2–based autonomy framework for agricultural UGVs and outdoor mobile robots, with a focus on GPS-centric navigation, field coverage, and simulation-first development.

## Project Goals

- GPS-based navigation in agricultural environments

- Coverage mission planning (e.g. lawnmower / field-row patterns)

- Simulation-first development using ROS 2 + Gazebo

- Clear upgrade path to RTK GNSS and multi-sensor fusion

## Current Scope

This repository focuses on the core autonomy stack for outdoor agricultural robots, including:

- Vehicle-agnostic localization and odometry

- Global pose estimation suitable for large outdoor fields

- Path-following controllers for UGV motion

- Integration-ready architecture for Nav2 and GNSS-based navigation

- Initial development targets GPS + wheel odometry localization in simulation, followed by higher-precision localization methods.

## Implemented Features ✅
- Localization & Odometry (Stable)

- Skid-steer wheel odometry computed from joint states

- EKF-based state estimation (robot_localization)

- Correct TF tree:
```bash
map → odom → base_link
```

- Nav2 is integrated for autonmous navigation

## Technologies

- ROS 2 (Humble)

- robot_localization (EKF)

- Gazebo (simulation-first workflow)

- GNSS (GPS, RTK-ready)

- Sensor fusion (IMU, LiDAR, Camera – planned)

## Project Status

🟡 Localization & Odometry Stable
🚧 Navigation stack integration (Nav2) Done
🚧 Gazebo + ros2_control integration planned

## ▶️ How to Run

### 1️⃣ Build the workspace

```bash
cd ~/agri_autonomy_ros2/Agri_Autonomy_Ros2/ros2_ws
colcon build
source install/setup.bash
```

---

### 2️⃣ Launch robot bringup (localization + mission planner)

This launches the **core system**:

* Robot description & TF
* Wheel odometry
* EKF localization (`map → odom → base_link`)
* Mission planner (starts and waits for Nav2)

```bash
ros2 launch agri_ugv_bringup bringup.launch.py
```

At this point:

* The robot is localized and visible in RViz
* `/mission/path` is published (coverage plan)
* The mission planner is **idle and waiting** for Nav2’s action server
* The robot **will not move yet**

---

### 3️⃣ Launch Nav2 (navigation stack)

Nav2 is launched **separately** and connects to the already-running system.

```bash
ros2 launch agri_nav2_bringup nav2.launch.py
```

Once Nav2 starts:

* The mission planner detects the Nav2 action server
* Coverage navigation begins automatically
* Goals are sent sequentially
* The robot starts moving

---

## 🧭 RViz Visualization

Recommended RViz displays:

* **RobotModel** → `/robot_description`
* **TF** → verify `map → odom → base_link`
* **Path** → `/mission/path` 
* **Odometry** → `/odometry/filtered`

---

## 🧩 Design Notes

* Nav2 is **not part of bringup by design**
* Bringup remains lightweight and reusable
* Mission logic is decoupled from navigation
* Nav2 can be replaced, tuned, or disabled without changing bringup



