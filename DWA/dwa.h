//
// Created by xcmworkharder on 2022/07/24
//

#ifndef DWA_CPU_DEMO_DWA_H
#define DWA_CPU_DEMO_DWA_H

#include "utility.h"

class Dwa {
public:
	Dwa(const State &start, const Point &goal, 
		const Obstacle &obs, const Config &config);
	bool StepOnceToGoal(std::vector<State> *best_trajectory, State *cur_state);

private:
	State Motion(const State &x, const Control &u, float dt);
	Window CalcDynamicWindow(const State &x);
	Traj CalcTrajectory(const State &x, float v, float w);
	float CalcObstacleCost(const Traj &traj, const Obstacle &obs);
	float CalcToGoalCost(const Traj &traj, const Point& goal);
	Traj CalcFinalInput(const State &x, Control &u, const Window &dw,
		                const Point &goal, const std::vector<Point> &obs);

	Point goal_;
	Obstacle obs_;
	Config config_;
	State cur_x_;
};

#endif // DWA_CPU_DEMO_DWA_H