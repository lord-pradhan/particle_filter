#define PI 3.1416 

class SensorModel{
private:
	MapReader map_obj;
	vector<vector<int>> occupancyMap;
	int resolution;

	// params
	int max_range;
	double laserOffset;
	// tunable params
	double gauss_sd, lambda_short;
	double wt_gauss, wt_short, wt_max, wt_rand, truncate_gauss;

public:
	SensorModel(const MapReader& map_objIn): map_obj(map_objIn){
		max_range = 8183; laserOffset = 25.0;
		gauss_sd = 30.0; lambda_short = 0.1;
		wt_gauss=10.0; wt_short=0.01; wt_max=0.001; wt_rand=0.00001;
		truncate_gauss = 3.0;

		resolution = map_obj.get_map_resolution();
		occupancyMap = transformoccupancymap(map_obj.get_map());
	}

	double calcCDF(double x){
		// constants
	    double a1 =  0.254829592;
	    double a2 = -0.284496736;
	    double a3 =  1.421413741;
	    double a4 = -1.453152027;
	    double a5 =  1.061405429;
	    double p  =  0.3275911;

	    // Save the sign of x
	    int sign = 1;
	    if (x < 0)
	        sign = -1;
	    x = fabs(x)/sqrt(2.0);

	    // A&S formula 7.1.26
	    double t = 1.0/(1.0 + p*x);
	    double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

	    // cout<<"cdf value is "<<0.5*(1.0 + sign*y)<<endl;
	    return 0.5*(1.0 + sign*y);
	}

	double beam_range_finder_model(const vector<int>& z_t1_arr, const odomMsg& x_t1){
				
		// initialize stuff for raycasting		
		odomMsg laserPose( x_t1.x + laserOffset*cos(x_t1.theta), x_t1.y + laserOffset*sin(x_t1.theta), 
						x_t1.theta );
		double startAbsX = laserPose.x, startAbsY = laserPose.y; 

		// initialize stuff for prob distributions		
		double log_p_tot=0.0;
		double p_tot = 1.0;

		// iterate through num of lasers
		int numLasers = z_t1_arr.size();		
		// cout<<"before entering loop for lasers in SensorModel. Num lasers is "<<numLasers<<endl;
		for(int i_las=0; i_las<numLasers; i_las++){

			double z_meas = z_t1_arr[i_las];	
			// cout<<"z measured is "<<z_meas<<endl;		

			/* Raycasting operation */
			double dir = laserPose.theta - PI/2.0 + (double) i_las * PI / numLasers;
			double dirX = cos(dir), dirY = sin(dir);
			double signDirX = dirX>0 ? 1 : -1;
			double signDirY = dirY>0 ? 1 : -1;
			// cout<<"signDirX is "<<signDirX<<", signDirY is "<<signDirY<<", dirX is "<<dirX<<", dirY is "<<dirY<<endl;

			double t=0.0, z_true = 0.0; bool collided = false, skip_gauss = false;
			int tileX, tileY;
			double currAbsX = startAbsX, currAbsY = startAbsY;
			absToTile(tileX, tileY, currAbsX, currAbsY, resolution);

			// cout<<"before entering raycasting while loop, tileX is "<<tileX<<", tileY is "<<tileY<<endl;
			while( inRangeMap(tileX, tileY, occupancyMap) && occupancyMap[tileY][tileX]==0 ){

				double dtX = fabs(dirX) > 1e-5 ? ( (tileX+signDirX)*resolution - currAbsX )/dirX : std::numeric_limits<double>::max();
				double dtY = fabs(dirY) > 1e-5 ? ( (tileY+signDirY)*resolution - currAbsY )/dirY : std::numeric_limits<double>::max();
				double dt;

				if(dtX < dtY){
					dt = dtX;
					tileX += signDirX;
				}
				else if(dtY < dtX){
					dt = dtY;
					tileY += signDirY;
				}
				else{
					dt = dtX; tileX += signDirX; tileY += signDirY;
				}

				t += dt;
				currAbsX = startAbsX + t*dirX; currAbsY = startAbsY + t*dirY;
				// cout<<"t is "<<t<<", currAbsX is "<<currAbsX<<", currAbsY is "<<currAbsY<<endl;
				z_true = t; 

				if(z_true > (max_range + truncate_gauss*gauss_sd)){
					skip_gauss = true;
					break;
				}

			}

			/* Calculate probability */
			double p_gauss, p_short, p_max, p_rand;

			// Gaussian distribution
			if(skip_gauss || z_meas > (double) max_range || z_meas<=0.0)
				p_gauss=0.0;
			else{
				// calc normalizer
				double llim = (0.0 - z_true)/gauss_sd;
				double ulim = ((double)max_range - z_true)/gauss_sd;

				double norm_gauss = 1.0/( calcCDF(ulim) - calcCDF(llim) );				
				// double norm_gauss = 1.0;

				p_gauss = norm_gauss * exp( -(z_meas - z_true)*(z_meas - z_true)/(2.0*gauss_sd*gauss_sd) ) 
							/ (gauss_sd*sqrt(2.0*PI))  ;
				// cout<<"norm_gauss is "<< norm_gauss<<" p_gauss is "<<p_gauss<<endl;
			}

			// exp distribution
			double norm_exp = 1.0 / (1.0 - exp(-lambda_short*z_true));
			if(z_meas <= z_true && z_meas>=0.0){
				p_short = norm_exp*lambda_short*exp(-lambda_short*z_meas);
			}
			else
				p_short = 0.0;
			// cout<<"p_short is "<<p_short<<endl;

			// max distribution
			if(fabs(z_meas - (double)max_range) < 2.0)
				p_max = 1.0;
			else
				p_max=0.0;
			// cout<<"p_max is "<<p_max<<endl;

			// random distribution
			if(z_meas>=0.0 && z_meas<= (double) max_range)
				p_rand = 1./(double)max_range;
			else
				p_rand=0.0;
			// cout<<"p_rand is "<<p_rand<<endl;			

			double wt_norm = wt_gauss + wt_short + wt_max + wt_rand;
			p_tot *= (wt_gauss/wt_norm*p_gauss + wt_short/wt_norm*p_short + wt_max/wt_norm*p_max + 
					wt_rand/wt_norm*p_rand);
			// cout << "p_tot interm is "<<p_tot<<endl;

			log_p_tot +=  log(wt_gauss/wt_norm*p_gauss + wt_short/wt_norm*p_short + 
						wt_max/wt_norm*p_max + wt_rand/wt_norm*p_rand);

			double log_p_temp = log(wt_gauss/wt_norm*p_gauss + wt_short/wt_norm*p_short + 
						wt_max/wt_norm*p_max + wt_rand/wt_norm*p_rand);

			// cout<<"z_true is "<<z_true<<", z_meas is "<<z_meas<< ", p_gauss is "<<p_gauss<<", p_short is "<<p_short<<", p_max is "<<p_max<<", p_rand is "<<p_rand<< ", log_p_temp is "<<log_p_temp<<endl;
			// cout<< "iterated probability is "<<(wt_gauss/wt_norm*p_gauss + wt_short/wt_norm*p_short + wt_max/wt_norm*p_max + wt_rand/wt_norm*p_rand) << endl;
		}
		
		// cout<<"returned log probability is "<< log_p_tot<<endl;
		// cout<<"returned probability is "<< p_tot<<endl;
		return log_p_tot;	
	}

	void raycasting(vector<odomMsg>& rays, odomMsg& x_t1){
				
		// initialize stuff for raycasting		
		odomMsg laserPose( x_t1.x + laserOffset*cos(x_t1.theta), x_t1.y + laserOffset*sin(x_t1.theta), 
						x_t1.theta );
		double startAbsX = laserPose.x, startAbsY = laserPose.y; 

		// iterate through num of lasers
		int numLasers = 180;	

		for(int i_las=0; i_las<numLasers; i_las++)
		{

			/* Raycasting operation */
			double dir = laserPose.theta - PI/2.0 + (double) i_las * PI / numLasers;
			double dirX = cos(dir), dirY = sin(dir);
			double signDirX = dirX>0 ? 1 : -1;
			double signDirY = dirY>0 ? 1 : -1;
			// cout<<"signDirX is "<<signDirX<<", signDirY is "<<signDirY<<", dirX is "<<dirX<<", dirY is "<<dirY<<endl;

			double t=0.0, z_true = 0.0; bool collided = false, skip_gauss = false;
			int tileX, tileY;
			double currAbsX = startAbsX, currAbsY = startAbsY;
			absToTile(tileX, tileY, currAbsX, currAbsY, resolution);

			// cout<<"before entering raycasting while loop, tileX is "<<tileX<<", tileY is "<<tileY<<endl;
			while( inRangeMap(tileX, tileY, occupancyMap) && occupancyMap[tileY][tileX]==0 )
			{

				double dtX = fabs(dirX) > 1e-5 ? ( (tileX+signDirX)*resolution - currAbsX )/dirX : std::numeric_limits<double>::max();
				double dtY = fabs(dirY) > 1e-5 ? ( (tileY+signDirY)*resolution - currAbsY )/dirY : std::numeric_limits<double>::max();
				double dt;

				if(dtX < dtY){
					dt = dtX;
					tileX += signDirX;
				}
				else if(dtY < dtX){
					dt = dtY;
					tileY += signDirY;
				}
				else{
					dt = dtX; tileX += signDirX; tileY += signDirY;
				}

				t += dt;
				currAbsX = startAbsX + t*dirX; currAbsY = startAbsY + t*dirY;
				z_true = t; 

				if(z_true > (max_range + truncate_gauss*gauss_sd)){
					break;
				}

			}
			rays.push_back(odomMsg(currAbsX, currAbsY, 0));
		}
	}
};