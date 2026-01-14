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

- Fake localization fully removed

- Architecture ready for GPS, RTK, and IMU fusion

- Motion Control (Baseline)

- Custom pure pursuit–style path follower

- Designed for low-speed agricultural UGV motion

- Known oscillation limits documented (controller to be replaced by Nav2)

⚠️ Note: The current controller is intentionally simple and used for early validation.
Nav2’s Regulated Pure Pursuit controller is planned for production navigation.

## Technologies

- ROS 2 (Humble)

- robot_localization (EKF)

- Gazebo (simulation-first workflow)

- GNSS (GPS, RTK-ready)

- Sensor fusion (IMU, LiDAR, Camera – planned)

## Project Status

🟡 Localization & Odometry Stable
🚧 Navigation stack integration (Nav2) in progress
🚧 Gazebo + ros2_control integration planned

This repository is under active development, with a focus on correctness, realism, and scalability to real agricultural robots.

