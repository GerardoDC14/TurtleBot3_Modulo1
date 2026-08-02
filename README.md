# TurtleBot3 Module 1 — Cooperative Control Experiments

![Status](https://img.shields.io/badge/status-academic%20experiment-blue?style=flat-square)
![ROS2](https://img.shields.io/badge/ROS%202-Humble-22314E?style=flat-square&logo=ros&logoColor=white)
![MATLAB](https://img.shields.io/badge/MATLAB-experiments-orange?style=flat-square)

Academic workspace for experimenting with TurtleBot3 control, orientation sensing, multi-robot formation concepts and MATLAB–ROS 2 communication.

## Main experiments

The MATLAB scripts cover several stages of the work:

- commanding one or two TurtleBot3 platforms;
- publishing emergency or complete-stop velocity commands;
- reading IMU orientation from ROS 2;
- sending orientation data to an ESP32 over TCP;
- testing leader–follower and formation-control ideas;
- implementing PID-based motion experiments;
- comparing physical and URDF-based simulation workflows.

## ROS 2 package

The workspace includes the Python package `aruco_orientation`, with an executable named `odom_node`. It supports the orientation/odometry side of the experiments and is built with `ament_python`.

## Repository structure

```text
TurtleBot3_Modulo1/
├── CompleteStop.m
├── IMUMatlab.m
├── Test1.m ... Test5.m
├── Test4_OneRobot.m
├── Test4_Simulation.m
├── Test4_Simulation_urdf.m
├── src/
│   └── aruco_orientation/
│       ├── package.xml
│       ├── setup.py
│       └── aruco_orientation/odom_node.py
├── build/                 # Historical colcon output
├── install/               # Historical colcon output
└── log/                   # Historical colcon logs
```

## Status

**Academic experiment / historical workspace.** The generated `build`, `install` and `log` directories are retained as part of the original snapshot but are not required to understand the source implementation.
