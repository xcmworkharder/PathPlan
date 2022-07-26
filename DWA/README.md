# DWA_Planner in CPU

## Enviroment
- Ubuntu 18.04
- OpenCV 3.2.0

## Install and Build
- git clone git clone https://github.com/xcmworkharder/PathPlan.git
- cd DWA
- mkdir build && cd build
- cmake ..
- make

## Run
./dwa_cpu_demo

### Parameters
- max_speed = 1.0;
- min_speed = -0.5;
- max_yaw_rate = 40.0 * PI / 180.0;
- max_accel = 0.2;
- robot_radius = 1.0;
- max_dyaw_rate = 40.0 * PI / 180.0;
- v_reso = 0.01;
- yaw_rate_reso = 0.01 * PI / 180.0;
- dt = 0.1;
- predict_time = 2.0;
- to_goal_cost_gain = 2.0;
- speed_cost_gain = 1.0;