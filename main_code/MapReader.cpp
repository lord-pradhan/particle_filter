#include<vector>
#include <Eigen/Dense>


using namespace Eigen;
using namespace std;

class MapReader{
private:
	vector<vector<double>> prob;
	int resolution, size_x, size_y, global_mapsize_x, global_mapsize_y, autoshifted_x, autoshifted_y;

public:		
	MapReader(char* mapName){

		int map_success = read_beesoft_map(mapName, this);
	}

	void visualize_map(){

	}

	vector<vector<double>> get_map(){return occupancy_map;}	

	int get_map_size_x(){return size_x;}

	int get_map_size_y(){return size_y;}
};