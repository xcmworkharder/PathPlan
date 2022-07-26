#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "utility.h"
#include "dwa.h"


cv::Point2i cv_offset(float x, float y, int image_width = 2000, 
	int image_height = 2000) {
	cv::Point2i output;
	output.x = int(x * 100) + image_width / 2;
	output.y = image_height - int(y * 100) - image_height / 3;
	return output;
}

int main() {
	State start{0.0, 0.0, PI / 8.0, 0.0, 0.0};
	Point goal{10.0, 10.0};
	Obstacle obs {
		{-1, -1},
		{0, 2},
		{4.0, 2.0},
		{5.0, 4.0},
		{5.0, 5.0},
		{5.0, 6.0},
		{5.0, 9.0},
		{8.0, 9.0},
		{7.0, 9.0},
		{12.0, 12.0}
	};
	
	Control u{0.0, 0.0};
	Config config;
	Traj trajs;
	trajs.push_back(start);

	bool stop = false;

	cv::namedWindow("DWAPlan", cv::WINDOW_NORMAL);
	int count = 0;
	Dwa dwa_demo(start, goal, obs, config);
	cv::Mat final_canvas;
	Traj traj_single;
	State state;
	while (!dwa_demo.StepOnceToGoal(&traj_single, &state)) {
		trajs.push_back(state);

		// visualization
		cv::Mat bg(3500, 3500, CV_8UC3, cv::Scalar(255, 255, 255));
		cv::circle(bg, cv_offset(goal.x, goal.y, bg.cols, bg.rows), 
			30, cv::Scalar(255, 0, 0), 5);
		for (unsigned int i = 0; i < obs.size(); ++i) {
			cv::circle(bg, cv_offset(obs[i].x, obs[i].y, bg.cols, bg.rows),
				       20, cv::Scalar(0, 0, 0), -1);
		}
		for (unsigned int i = 0; i < traj_single.size(); ++i) {
			cv::circle(bg, cv_offset(traj_single[i].x, traj_single[i].y, 
						bg.cols, bg.rows), 7, cv::Scalar(0, 255, 0), -1);
		}
		cv::circle(bg, cv_offset(state.x, state.y, bg.cols, bg.rows), 
			       30, cv::Scalar(0, 0, 255), 5);

		cv::arrowedLine(
			bg,
			cv_offset(state.x, state.y, bg.cols, bg.rows),
			cv_offset(state.x + std::cos(state.theta), state.y + 
						std::sin(state.theta), bg.cols, bg.rows),
			cv::Scalar(255, 0, 255),
			7
			);

		cv::imshow("DWAPlan", bg);
		cv::waitKey(1);
		count++;
		final_canvas = bg;
	}

	if (!final_canvas.empty()) {
		for (unsigned int j = 0; j < trajs.size(); ++j) {
			cv::circle(final_canvas, cv_offset(trajs[j].x, trajs[j].y, 
						final_canvas.cols, final_canvas.rows), 
						7, cv::Scalar(0, 0, 255), -1);
		}

		cv::imshow("DWAPlan", final_canvas);
		cv::waitKey();
	}

	return 0;
}