/*
Low variance sampler implemented as in the Probabilistic Robotics book
*/
class Resampling{

public:
	Resampling(){}

	vector< wtOdomMsg > multinomial_sampler(vector< wtOdomMsg > X_t)
	{
		vector< wtOdomMsg > X_t_bar;
		return X_t_bar;
	}

	vector< wtOdomMsg > low_variance_sampler(vector< wtOdomMsg > &X_t)
	{	
		vector< wtOdomMsg > X_t_bar;
		int M = X_t.size();
		double M_inv = (1/(double)M);
		
		double r = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/M_inv));
		// cout << "r val: " << r << endl;
		double c = X_t[0].wt;
		// cout << "c val: " << c << endl;
		int i = 0;

		for(int m = 0; m < M; m++)
		{
			double u = r + (m - 1)*M_inv;
			while (u > c)
			{
				// cout << "comes" << endl;
				i = i + 1;
				c = c + X_t[i].wt;
			}
			
			X_t_bar.push_back(X_t[i]);
		}
		return X_t_bar;
	}
};