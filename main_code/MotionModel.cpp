/*
Odometry Motion Model from Probabilistic Robotics Chapter 5 
*/

class MotionModel{

private:
	
	double alpha1;
	double alpha2;
	double alpha3;
	double alpha4;

	std::default_random_engine generator;

public:

	MotionModel(float a1, float a2, float a3, float a4)
	{
		// Error Parameters
		alpha1 = a1;
		alpha2 = a2;
		alpha3 = a3;
		alpha4 = a4;
	}

	double normaldist(double mean, double stddev)
	{
    	std::normal_distribution<double> distribution(mean,stddev);
    	double sample = distribution(generator);
		return sample;
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

		double x_bar;
		double y_bar;
		double theta_bar;

		double x_bar_p;
		double y_bar_p;
		double theta_bar_p;

		double x;
		double y;
		double theta;

		x_bar = u_t0.x;
		y_bar = u_t0.y;
		theta_bar = u_t0.theta;

		x_bar_p = u_t1.x;
		y_bar_p = u_t1.y;
		theta_bar_p = u_t1.theta;

		x = x_t0.x;
		y = x_t0.y;
		theta = x_t0.theta;

		delta_rot1 = atan2(y_bar_p - y_bar, x_bar_p - x_bar) - theta_bar;
		delta_trans = sqrt((x_bar_p - x_bar)*(x_bar_p - x_bar) + (y_bar_p - y_bar)*(y_bar_p - y_bar));
		delta_rot2 = theta_bar_p - theta_bar - delta_rot1;

		// cout << delta_trans << endl;

		delta_rot1_hat = delta_rot1 - normaldist(0.0,alpha1*abs(delta_rot1) + alpha2*abs(delta_trans));
		delta_trans_hat = delta_trans - normaldist(0.0,alpha3*abs(delta_trans) + alpha4*abs(delta_rot1 + delta_rot2));
		delta_rot2_hat = delta_rot2 - normaldist(0.0,alpha1*abs(delta_rot2) + alpha2*abs(delta_trans));

		x_p = x + delta_trans_hat*cos(theta+delta_rot1_hat);
		y_p = y + delta_trans_hat*sin(theta+delta_rot1_hat);
		theta_p = theta + delta_rot1_hat + delta_rot2_hat;

		// cout << x << " " << x_p << endl;
		// cout << y << " " << y_p << endl;
		// cout << endl;

		odomMsg x_t(x_p, y_p, theta_p);

		return x_t;
	}
};

// int main()
// {
// 	return 0;
// }