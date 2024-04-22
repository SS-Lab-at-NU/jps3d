#include <vector>
#include <jps_basis/data_utils.h>
#include <jps_planner/jps_planner/jps_planner.h>
#include <jps_planner/distance_map_planner/distance_map_planner.h>

std::vector<Vec2f> plan(const Vec2f &origin, const Vec2f &start, const Vec2f &goal, const std::vector<std::vector<float>> &map, const float &map_resolution, bool verbose = false)
{
	std::vector<signed char> serialized_map;
	for(int i = 0; i < map[0].size(); i++)
	{
		for(int j = 0; j < map.size(); j++)
		{
			serialized_map.push_back(map[j][i]);
		}
	}

	std::shared_ptr<JPS::OccMapUtil> map_util = std::make_shared<JPS::OccMapUtil>();
	
	map_util->setMap(origin, {map.size(), map[0].size()}, serialized_map, map_resolution);

	if(verbose)
	{
		std::cout << start[0] << " " << start[1] << std::endl;
		std::cout << goal[0] << " " << goal[1] << std::endl;
		std::cout << map.size() << " " << map[0].size() << std::endl;
		for(int i = 0; i < map.size(); i++){
			for(int j = 0; j < map[0].size(); j++){
				if(i == start[0] && j == start[1])
					std::cout << "S" << " ";
				else if(i == goal[0] && j == goal[1])
					std::cout << "G" << " ";
				else if(i == origin[0] && j == origin[1])
				    std::cout << "*" << " ";
				else
					std::cout << map_util->isOccupied(map_util->getIndex({i,j})) << " ";
			}
			std::cout << std::endl;
		}
	}

	JPSPlanner2D jps_planner(verbose);
	jps_planner.setMapUtil(map_util);
	jps_planner.updateMap();

	bool valid_jps = jps_planner.plan(start, goal, 1, true);
	const auto path_jps = jps_planner.getRawPath(); // Get the planned raw path from JPS
	if(verbose)
	{
		if(valid_jps){
			printf("JPS Path Distance: %f\n", total_distance2f(path_jps));
			std::cout << "JPS Path size = " << path_jps.size() << std::endl;
		}
		else{
			printf("Could not find a valid JPS path\n");
		}
	}

	// Set up DMP planner
	DMPlanner2D dmp(true);
	dmp.setPotentialRadius(Vec2f(0.5, 0.5)); // Set 2D potential field radius
	dmp.setSearchRadius(Vec2f(15, 15)); // Set the valid search region around given path
	dmp.setMap(map_util, start); // Set map util for collision checking, must be called before planning
	bool valid_dist = dmp.computePath(start, goal, path_jps); // Compute the path given the jps path
	const auto path_dist = dmp.getRawPath();
	if(valid_dist){
	    printf("DMP Path Distance: %f\n", total_distance2f(path_dist));
		std::cout << "Path size = " << path_dist.size() << std::endl;
		}
	else{
	    printf("Could not find a valid DMP path\n");
		}

    std::vector<Vec2f> output_path;
	for(int i = 0; i < path_jps.size(); i++){
		output_path.push_back(path_jps[i]);
    }
	return output_path;
}