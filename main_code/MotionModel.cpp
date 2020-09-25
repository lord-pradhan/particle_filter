#include <cmath>
#include "utils.h"
/*
Odometry Motion Model from Probabilistic Robotics Chapter 5 
*/

class MotionModel{

	MotionModel(float a1, float a2, float a3, float a4)
	{
		// Error Parameters
		float alpha1 = a1;
		float alpha2 = a2;
		float alpha3 = a3;
		float alpha4 = a4;
	}

	odomMsg update(odomMsg u_t0, odomMsg u_t1, odomMsg x_t0)
	{
		double delta_rot1;
		double delta_trans;
		double delta_rot2;

		double delta_rot1_hat;
		double delta_trans_hat;
		double delta_rot2_hat;

		double x_p;
		double y_p;
		double theta_p;

		x_t = odomMsg();
		x_t.x = x_p;
		x_t.y = y_p;
		x_t.theta = theta_p;

		return x_t; 
	}
};