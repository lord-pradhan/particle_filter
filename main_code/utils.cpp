

struct odomMsg
{
	double x, y, theta;
	odomMsg(double x_init, double y_init, double theta_init)
	{
		x = x_init;
		y = y_init;
		theta = theta_init;
	}
};

struct wtOdomMsg
{
	double x, y, theta;
	wtOdomMsg(double x_init, double y_init, double theta_init)
	{
		x = x_init;
		y = y_init;
		theta = theta_init;
	}
};