#include <math.h>

// helper functions
void tileToAbs(int tileX, int tileY, double& absX, double& absY, int resolution){
	// absX = tileX*resolution
}


void absToTile(int& tileX, int& tileY, double absX, double absY, int resolution){
	tileX = floor((double) absX/resolution); tileY = floor((double)  absY/resolution);
}


class odomMsg
{
public:
	double x, y, theta;

	odomMsg(){}

	odomMsg(double x_init, double y_init, double theta_init)
	{
		x = x_init;
		y = y_init;
		theta = theta_init;
	}
};

class wtOdomMsg
{
public:
	double x, y, theta, wt;

	wtOdomMsg(){}

	wtOdomMsg(odomMsg odomIn, double wtIn){
		x = odomIn.x; y = odomIn.y; z = odomIn.z; wt = wtIn;
	}

	wtOdomMsg(double x_init, double y_init, double theta_init, double wt_init){
		x = x_init; y = y_init; theta = theta_init; wt = wt_init;
	}
};