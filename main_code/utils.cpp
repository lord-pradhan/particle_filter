#include <math.h>

// helper functions
void tileToAbs(int tileX, int tileY, double& absX, double& absY, int resolution){
	// absX = tileX*resolution
}

void absToTile(int& tileX, int& tileY, double absX, double absY, int resolution){
	tileX = floor((double) absX/resolution); tileY = floor((double)  absY/resolution);
}

bool inRangeMap(int tileX, int tileY, const vector<vector<int>>& mapIn){
	bool returnVal = true;
	int Ymax = mapIn.size(), Xmax = mapIn[0].size();
	
	if(tileX<0 || tileX>Xmax || tileY<0 || tileY>Ymax) returnVal = false;

	return returnVal;
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
		x = odomIn.x; y = odomIn.y; theta = odomIn.theta; wt = wtIn;
	}

	wtOdomMsg(double x_init, double y_init, double theta_init, double wt_init){
		x = x_init; y = y_init; theta = theta_init; wt = wt_init;
	}
};

vector<vector<int>> transformoccupancymap(const vector<vector<int>>& map)
{
	vector<vector<int>> transformedmap(map.size(), vector<int>(map[0].size(), -1));	
	for(int i = 0; i < map.size(); i++)
		for(int j = 0; j < map[0].size(); j++)
			transformedmap[i][j] = map[map.size()-1-i][j];
	return transformedmap;
}