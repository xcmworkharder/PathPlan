//
// Created by xcmworkharder 2022/07/24
//

#ifndef DWA_CPU_DEMO_UTILITY_H
#define DWA_CPU_DEMO_UTILITY_H

#include <vector>

#define PI 3.14159265358

struct State {
	float x;
	float y;
	float theta;
	float v;
	float w;
};

using Traj = std::vector<State>;

struct Window {
	float min_v;
	float max_v;
	float min_w;
	float max_w;
};

struct Control {
	float v;
	float w;
};

struct Point {
	float x;
	float y;
};

using Obstacle = std::vector<Point>;

struct Config {
	float max_speed = 1.0;
	float min_speed = -0.5;
	float max_yaw_rate = 40.0 * PI / 180.0;
	float max_accel = 0.2;
	float robot_radius = 1.0;
	float max_dyaw_rate = 40.0 * PI / 180.0;

	float v_reso = 0.01;
	float yaw_rate_reso = 0.01 * PI / 180.0;

	float dt = 0.1;
	float predict_time = 2.0;
	float to_goal_cost_gain = 2.0;
	float speed_cost_gain = 1.0;
};

#endif // DWA_CPU_DEMO_UTILITY_H