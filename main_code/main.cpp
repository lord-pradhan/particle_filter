// Header files

void visualize_map(){}

void visualize_timestep(X_bar, tstep){}

vector<wtOdomMsg> init_particles_random(int num_particles, <insert> occupancy_map){


}

vector<wtOdomMsg> init_particles_freespace(int num_particles, vector<vector<double>> occupancy_map){

}

int main(){

	char* path_map = '../data/map/wean.dat';
	char* path_log = '../data/log/robotdata1.log';

	MapReader map_obj = MapReader(path_map);
	vector<vector<double>> occupancy_map = map_obj->get_map();
	// logfile

	MotionModel motion();
	SensorModel sensor();
	Resampling resampler();

	int num_particles = 500;
	vector< wtOdomMsg > init_particles_random(num_particles, occupancy_map);

	bool vis_flag = true;

	if(vis_flag)
		visualize_map(occupancy_map);

	bool first_time_idx = true;

	int logSize = logfile.size();
	for(int i=0; i<logSize; i++){

		// read log file
		auto line = logfile[i];
		
	}
}