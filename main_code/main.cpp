// Header files
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <random>
#include <stdio.h>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

#include "MapReader.cpp"
#include "utils.cpp"

void visualize_map_with_particles(MapReader &map_obj, vector<wtOdomMsg> &particles)
{
	Mat imgwithparticles;
	imgwithparticles = map_obj.get_img().clone();
	for(wtOdomMsg particle : particles)
	{
		double x = particle.x/map_obj.get_map_resolution();
		double y = map_obj.get_map_size_y() - particle.y/map_obj.get_map_resolution();
		imgwithparticles.at<Vec3b>(y,x) = Vec3b(0,0,255);
	}
	map_obj.visualize_map(imgwithparticles);
}

// void visualize_timestep(X_bar, tstep){}

vector<wtOdomMsg> init_particles_random(int num_particles, MapReader &map_obj){

	// Create a random device and use it to generate a random seed
    std::random_device myRandomDevice;
    unsigned seed = myRandomDevice();
    
    // Initialize a default_random_engine with the seed
    std::default_random_engine myRandomEngine(seed);

    // Initialize a uniform_real_distribution to produce values
    std::uniform_real_distribution<double> y0_unif_dist(0.0, 7500.0), x0_unif_dist(3000.0, 7000.0), 
    	theta0_unif_dist(-3.14, 3.14);

	vector<wtOdomMsg> X_bar_init;//y0_vals, x0_vals, theta0_vals;

	for(int i=0; i<num_particles; i++){

		wtOdomMsg temp(x0_unif_dist(myRandomEngine), y0_unif_dist(myRandomEngine), 
			theta0_unif_dist(myRandomEngine), 1.0/num_particles);

		X_bar_init.push_back(temp);
	}

	return X_bar_init;
}

vector<wtOdomMsg> init_particles_freespace(int num_particles, MapReader &map_obj){

	// Create a random device and use it to generate a random seed
    std::random_device myRandomDevice;
    unsigned seed = myRandomDevice();
    
    // Initialize a default_random_engine with the seed
    std::default_random_engine myRandomEngine(seed);

    // Initialize a uniform_real_distribution to produce values
    std::uniform_real_distribution<double> y0_unif_dist(0.0, 7500.0), x0_unif_dist(3000.0, 7000.0), 
    	theta0_unif_dist(-3.14, 3.14);

	vector<wtOdomMsg> X_bar_init;//y0_vals, x0_vals, theta0_vals;

	vector<vector<double>> occupancy_map = map_obj.get_map_prob();
	int res = map_obj.get_map_resolution();

	for(int i=0; i<num_particles; i++){

		wtOdomMsg temp(x0_unif_dist(myRandomEngine), y0_unif_dist(myRandomEngine), 
			theta0_unif_dist(myRandomEngine), 1.0/num_particles);
		double pr = occupancy_map[map_obj.get_map_size_y() - 1 - floor(temp.y/res)][floor(temp.x/res)]; 
		if (pr >= 0 && pr <= 0.1)
			X_bar_init.push_back(temp);
		else
			i--;
	}

	return X_bar_init;
}

int main(){

	const char* path_map = "data/map/wean.dat";
	const char* path_log = "data/log/robotdata1.log";

	MapReader map_obj = MapReader(path_map);
	vector<vector<double>> occupancy_map = map_obj.get_map();

	int num_particles = 5000;
	// vector<wtOdomMsg> particles = init_particles_random(num_particles, map_obj);
	vector<wtOdomMsg> particles = init_particles_freespace(num_particles, map_obj);

	bool vis_flag = true;

	if(vis_flag)
		visualize_map_with_particles(map_obj, particles);

	// // initialize log file
	// std::string logfile;
	// std::ifstream infile(path_log);

	// MotionModel motion();
	// SensorModel sensor();
	// Resampling resampler();

	// bool vis_flag = true;

	// if(vis_flag)
	// 	visualize_map(occupancy_map);

	// bool first_time_idx = true;

	// // parse log file
	// std::string line;
	// int time_step=0;
	// while(std::getline(infile, line)){

	// 	time_step++;

	// 	std::istringstream iss(line);
	// 	char meas_type;

	// 	if(!(iss >> meas_type)) { cout<<"Error while reading in meas_type"<<endl; break; }

	// 	vector<double> meas_vals;
	// 	double tempStream;

	// 	while(iss >> tempStream)
	// 		meas_vals.push_back(tempStream);

	// 	cout<<"finished streaming"<<endl;

	// 	odomMsg odometry_robot(meas_vals[0], meas_vals[1], meas_vals[2]);

	// 	double time_stamp = meas_vals.back();

	// 	vector<int> ranges;
	// 	if(meas_type=='L'){
	// 		odomMsg odometry_laser(meas_vals[3], meas_vals[4], meas_vals[5]);
	// 		int rangeSize = meas_vals.size();			

	// 		for(int i=6; i<rangeSize-1; i++) 
	// 			ranges.push_back(meas_vals[i]);			
	// 	}

	// 	cout<< "Processing time step "<< time_step <<" at time "<<time_stamp<<"s"<<endl;

	// 	if(first_time_idx){

	// 		auto u_t0 = odometry_robot;
	// 		first_time_idx = false;
	// 		continue;
	// 	}

	// 	vector<wtOdomMsg> X_bar_new = X_bar;
	// 	auto u_t1 = odometry_robot;

	// 	for(int i=0; i<num_particles; i++){

	// 		// Motion model
	// 		odomMsg x_t0(X_bar[i].x, X_bar[i].y, X_bar[i].theta);
	// 		odomMsg x_t1 = motion.update(u_t0, u_t1, x_t0);

	// 		// Sensor model
	// 		if(meas_type=='L'){
	// 			vector<int> z_t = ranges;
	// 			double w_t = sensor.beam_range_finder_model(z_t, x_t1);

	// 			X_bar_new[i] = wtOdomMsg(x_t1, w_t);
	// 		}
	// 		else{
	// 			X_bar_new[i] = wtOdomMsg(x_t1, X_bar[i].wt);
	// 		}			
	// 	}

	// 	X_bar = X_bar_new;
	// 	u_t0 = u_t1;

	// 	// Resampling
	// 	X_bar = resampler.low_variance_sampler(X_bar);

	// 	if(vis_flag)
	// 		visualize_timestep(X_bar,time_step);
	// }

	return 0;
}