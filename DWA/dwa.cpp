// 
// Created by xcmworkharder on 2022/07/24
//

#include "dwa.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <array>
#include <limits>

Dwa::Dwa(const State &start, const Point &goal,	const Obstacle &obs,
	const Config &config) : cur_x_(start), goal_(goal), 
	obs_(obs), config_(config) {

}

bool Dwa::StepOnceToGoal(std::vector<State> *best_trajectory, 
	State *cur_state) {
	Control calc_u;
	Window dw = CalcDynamicWindow(cur_x_);
	Traj traj_list = CalcFinalInput(cur_x_, calc_u, dw, goal_, obs_);
	cur_x_ = Motion(cur_x_, calc_u, config_.dt);

	*best_trajectory = traj_list;
	*cur_state = cur_x_;
	if (std::sqrt(std::pow((cur_x_.x - goal_.x), 2) + 
		std::pow((cur_x_.y - goal_.y), 2)) <= config_.robot_radius) {
		return true;
	}
	return false;
}

State Dwa::Motion(const State &x, const Control &u, float dt) {
	State new_state = x;
	new_state.theta += u.w * dt;
	new_state.x += u.v * std::cos(new_state.theta) * dt;
	new_state.y += u.v * std::sin(new_state.theta) * dt;
	new_state.v = u.v;
	new_state.w = u.w;
	return new_state;
}

Window Dwa::CalcDynamicWindow(const State &x) {
	return {
		std::max(x.v - config_.max_accel * config_.dt, config_.min_speed),
		std::min(x.v + config_.max_accel * config_.dt, config_.max_speed),
		std::max(x.w - config_.max_dyaw_rate * config_.dt, 
			-config_.max_yaw_rate),
		std::min(x.w + config_.max_dyaw_rate * config_.dt, 
			config_.max_yaw_rate)
	};
}

Traj Dwa::CalcTrajectory(const State &x, float v, float w) {
	Traj traj;
	traj.push_back(x);
	float time = 0;

	State cur_state = x;
	while (time <= config_.predict_time) {
		cur_state = Motion(cur_state, Control{v, w}, config_.dt);
		traj.push_back(cur_state);
		time += config_.dt;
	}
	return traj;
}

float Dwa::CalcObstacleCost(const Traj &traj, const Obstacle &obs) {
	float min_radius = std::numeric_limits<float>::max();

	for (unsigned int i = 0; i < traj.size(); ++i) {
		for (unsigned int j = 0; j < obs.size(); ++j) {
			float ox = obs[j].x;
			float oy = obs[j].y;
			float dx = traj[i].x - ox;
			float dy = traj[i].y - oy;

			float r = std::sqrt(dx * dx + dy * dy);
			if (r <= config_.robot_radius) {
				return std::numeric_limits<float>::max();
			}

			if (min_radius >= r) {
				min_radius = r;
			}
		}
	}

	return 1.0 / min_radius;
}

float Dwa::CalcToGoalCost(const Traj &traj, const Point &goal) {
	float goal_magnitude = std::sqrt(goal.x * goal.x + goal.y * goal.y);
	float traj_magnitude = std::sqrt(std::pow(traj.back().x, 2) 
		+ std::pow(traj.back().y, 2));
	float dot_product = (goal.x * traj.back().x) + (goal.y * traj.back().y);
	float error = dot_product / (goal_magnitude * traj_magnitude);
	float error_angle = std::acos(error);
	float cost = config_.to_goal_cost_gain * error_angle;

	return cost;
}

Traj Dwa::CalcFinalInput(const State &x, Control &u, const Window &dw,
	                const Point &goal, const std::vector<Point> &obs) {
	float min_cost = std::numeric_limits<float>::max();
	Control min_u = u;
	Traj best_traj;

	clock_t start, stop;
	start = clock();
	for (float v = dw.min_v; v <= dw.max_v; v += config_.v_reso) {
		for (float w = dw.min_w; w <= dw.max_w; w += config_.yaw_rate_reso) {
			Traj traj = CalcTrajectory(x, v, w);

			float to_goal_cost = CalcToGoalCost(traj, goal);
			float speed_cost = config_.speed_cost_gain * 
								(config_.max_speed - traj.back().v);
			float obs_cost = CalcObstacleCost(traj, obs);
			float total_cost = to_goal_cost + speed_cost + obs_cost;

			if (min_cost >= total_cost) {
				min_cost = total_cost;
				min_u = Control{v, w};
				best_traj = traj;
			}
		}
	}
	stop = clock();
	float elapsedTime = (float)(stop - start) / 
						(float)CLOCKS_PER_SEC * 1000.0f;
	printf("Time to generate: %3.1f ms\n", elapsedTime);
	u = min_u;
	return best_traj;
}