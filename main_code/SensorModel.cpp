#define PI 3.1416 

class SensorModel{
private:
	MapReader map_obj;
	int resolution;

	// params
	int max_range;
	double laserOffset, gauss_sd, lambda_short;
	double wt_gauss, wt_short, wt_max, wt_rand;

public:
	SensorModel(const MapReader& map_objIn): map_obj(map_objIn){
		max_range = 8183; laserOffset = 25.0;
		gauss_sd = 1.0; lambda_short = 1.0;
		wt_gauss=1.0; wt_short=1.0; wt_max=1.0; wt_rand=1.0;
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

	    return 0.5*(1.0 + sign*y);
	}

	double beam_range_finder_model(vector<int> z_t1_arr, odomMsg x_t1){
				
		// initialize stuff for raycasting		
		odomMsg laserPose( x_t1.x + laserOffset*cos(x_t1.theta), x_t1.y + laserOffset*sin(x_t1.theta), 
						x_t1.theta );
		double startAbsX = laserPose.x, startAbsY = laserPose.y; 

		// initialize stuff for prob distributions		
		double p_tot=1.0;

		// iterate through num of lasers
		int numLasers = z_t1_arr.size();		
		for(int i_las=0; i_las<numLasers; i_las++){

			double z_meas = z_t1_arr[i_las];			

			/* Raycasting operation */
			double dir = laserPose.theta - PI/2.0 + (double) i_las * PI / numLasers;
			double dirX = cos(dir), dirY = sin(dir);
			double signDirX = dirX>0 ? 1 : -1;
			double signDirY = dirY>0 ? 1 : -1;

			double t=0.0; bool collided = false;
			int tileX, tileY;
			double currAbsX = startAbsX, currAbsY = startAbsY;
			absToTile(tileX, tileY, currAbsX, currAbsY, resolution);

			while(inRangeMap(tileX, tileY, map_obj.get_map()) && map_obj.get_map()[tileY][tileX]==0){								

				double dtX = fabs(dirX) > 1e-5 ? ( (tileX+1)*resolution - currAbsX )/dirX : std::numeric_limits<double>::max();
				double dtY = fabs(dirY) > 1e-5 ? ( (tileY+1)*resolution - currAbsY )/dirY : std::numeric_limits<double>::max();
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
			}

			double z_true = sqrt((currAbsX - startAbsX)*(currAbsX - startAbsX) + 
									(currAbsY - startAbsY)*(currAbsY - startAbsY)) ; 

			/* Calculate probability */
			double p_gauss, p_short, p_max, p_rand;

			// Gaussian distribution
			if(z_meas > (double) max_range)
				p_gauss=0.0;
			else{
				// calc normalizer
				double llim = (0.0 - z_true)/gauss_sd;
				double ulim = ((double)max_range - z_true)/gauss_sd;

				double norm_gauss = calcCDF(ulim) - calcCDF(llim);

				p_gauss = norm_gauss * exp( -(z_meas - z_true)*(z_meas - z_true)/(2.0*gauss_sd*gauss_sd) ) 
							/ (gauss_sd*sqrt(2.0*PI));
			}

			// exp distribution
			double norm_exp = 1.0 / (1.0 - exp(-lambda_short*z_true));
			if(z_meas <= z_true && z_meas>=0.0){
				p_short = norm_exp*lambda_short*exp(-lambda_short*z_meas);
			}
			else
				p_short = 0.0;

			// max distribution
			if(fabs(z_meas - (double)max_range)<1e-4)
				p_max = 1.0;
			else
				p_max=0.0;

			// random distribution
			if(z_meas>=0.0 && z_meas<= (double) max_range)
				p_rand = (double) 1.0/max_range;
			else
				p_rand=0.0;

			double wt_norm = wt_gauss + wt_short + wt_max + wt_rand;
			p_tot *= wt_gauss/wt_norm*p_gauss + wt_short/wt_norm*p_short + 
						wt_max/wt_norm*p_max + wt_rand/wt_max*p_max;
		}
		
		return p_tot;	
	}
};