class MapReader
{

private:

	typedef struct {
	  int resolution, size_x, size_y;
	  float offset_x, offset_y;
	  int min_x, max_x, min_y, max_y;
	  vector<vector<double>> prob;
	} map_type;

	map_type* map;
	vector<vector<int>> occupancy_map;
	Mat img;

public:

	MapReader(const char* mapName)
	{
		map = new map_type;
		int map_success = read_beesoft_map(mapName, map);

		img = Mat(map->size_y, map->size_x, CV_8UC3);

		for (int y = 0; y < map->size_y; y++)
		{
			for (int x = 0; x < map->size_x; x++)
			{
				double pr = map->prob[x][y];
				img.at<Vec3b>(x,y) = (pr >= 0) ? Vec3b((1-pr)*255,(1-pr)*255,(1-pr)*255) : Vec3b(255,0,0);
			}
		}

		vector< int > temp1(map->prob[0].size(), 0);
		vector<vector<int>> temp2(map->prob.size(), temp1);
		occupancy_map = temp2;
		
		for(int i = 0; i < map->prob.size(); i++)
			for(int j = 0; j < map->prob[0].size(); j++)
			{
				if (map->prob[i][j] < 0 || map->prob[i][j] > 0.2)
					occupancy_map[i][j] = 1;
				else
					occupancy_map[i][j] = 0;

			}
	}

	void visualize_map(Mat image)
	{
		const char* source_window = "Display Map";
		namedWindow( source_window );
		imshow(source_window, image);
		waitKey();
	}

	vector<vector<int>> get_map(){return occupancy_map;}
	vector<vector<double>> get_map_prob(){return map->prob;}
	int get_map_resolution(){return map->resolution;}
	int get_map_size_x(){return map->size_x;}
	int get_map_size_y(){return map->size_y;}
	Mat get_img(){return img;}

	int read_beesoft_map(const char *mapName, map_type *map)
	{
	  int x, y, count;
	  float temp;
	  char line[256];
	  FILE *fp;

	  if((fp = fopen(mapName, "rt")) == NULL) {
	    fprintf(stderr, "# Could not open file %s\n", mapName);
	    return -1;
	  }
	  fprintf(stderr, "\n# Reading map: %s\n", mapName);
	  while((fgets(line, 256, fp) != NULL)
		&& (strncmp("global_map[0]", line , 13) != 0)) {
	    if(strncmp(line, "robot_specifications->resolution", 32) == 0)
	      if(sscanf(&line[32], "%d", &(map->resolution)) != 0)
		printf("# Map resolution: %d cm\n", map->resolution);
	    if(strncmp(line, "robot_specifications->autoshifted_x", 35) == 0)
	      if(sscanf(&line[35], "%g", &(map->offset_x)) != 0) {
		map->offset_x = map->offset_x;
		printf("# Map offsetX: %g cm\n", map->offset_x);
	      }
	    if(strncmp(line, "robot_specifications->autoshifted_y", 35) == 0) {
	      if (sscanf(&line[35], "%g", &(map->offset_y)) != 0) {
		map->offset_y = map->offset_y;
		printf("# Map offsetY: %g cm\n", map->offset_y);
	      }
	    }
	  }

	  if(sscanf(line,"global_map[0]: %d %d", &map->size_y, &map->size_x) != 2) {
	    fprintf(stderr, "ERROR: corrupted file %s\n", mapName);
	    fclose(fp);
	    return -1;
	  }
	  printf("# Map size: %d %d\n", map->size_x, map->size_y);

	  map->prob = vector<vector<double>>(map->size_x, vector<double>(map->size_y,0));
	  map->min_x = map->size_x;
	  map->max_x = 0;
	  map->min_y = map->size_y;
	  map->max_y = 0;
	  count = 0;
	  for(x = 0; x < map->size_x; x++)
	    for(y = 0; y < map->size_y; y++, count++) {
	      if(count % 10000 == 0)
		fprintf(stderr, "\r# Reading ... (%.2f%%)",
			count / (float)(map->size_x * map->size_y) * 100);
	      
	      fscanf(fp,"%e", &temp);
	      if(temp < 0.0)
		map->prob[x][y] = -1;
	      else {
		if(x < map->min_x)
		  map->min_x = x;
		else if(x > map->max_x)
		  map->max_x = x;
		if(y < map->min_y)
		  map->min_y = y;
		else if(y > map->max_y)
		  map->max_y = y;
		map->prob[x][y] = 1 - temp;	   
	      }
	    }
	  fprintf(stderr, "\r# Reading ... (%.2f%%)\n\n",
		  count / (float)(map->size_x * map->size_y) * 100);
	  fclose(fp);
	  return 0;
	}
};

// int main(int argc, char **argv)
// {
//   MapReader mapobj(argv[1]);
//   mapobj.visualize_map();
//   return 0;
// }
