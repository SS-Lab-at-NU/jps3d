#include <iostream>
#include <jps_basis/data_utils.h>
#include <jps_planner/jps_planner/jps_planner.h>
#include <jps_planner/distance_map_planner/distance_map_planner.h>

int main(int argc, char *argv[]) {
	std::shared_ptr<JPS::OccMapUtil> map_util = std::make_shared<JPS::OccMapUtil>();
	map_util->setMap({-0.5,-0.5}, {2, 5}, {0, 0, 1, 0, 0, 0, 0, 1, 0, 0}, 1);
	const Vec2f start_pos(0, 0);
	const Vec2f goal_pos(1, 4);

	JPSPlanner2D jps(true); // Declare a planner
	jps.setMapUtil(map_util); // Set collision checking function
	jps.updateMap();

	for(int i = 0; i < 2; i++){
		for(int j = 0; j < 5; j++){
			if(i == start_pos(0) && j == start_pos(1))
				std::cout << "S" << " ";
			else if(i == goal_pos(0) && j == goal_pos(1))
				std::cout << "G" << " ";
			else
				std::cout << map_util->isOccupied(map_util->getIndex({i,j})) << " ";
        }
		std::cout << std::endl;
    }

	bool valid_jps = jps.plan(start_pos, goal_pos, 1, true);
	const auto path_jps = jps.getRawPath(); // Get the planned raw path from JPS
	if(valid_jps)
	    printf("JPS Path Distance: %f\n", total_distance2f(path_jps));
	else
		printf("Could not find a valid JPS path\n");

	std::cout << (path_jps.size() == 0? "Path is empty\n" : "Path is not empty. Path size = ") << path_jps.size() << std::endl;

	for(int i = 0; i < path_jps.size(); i++){
		std::cout << path_jps[i][0] << " " << path_jps[i][1] << std::endl;
	}

	std::cout << std::endl;

	// Set up DMP planner
	DMPlanner2D dmp(true);
	dmp.setPotentialRadius(Vec2f(0.1, 0.1)); // Set 2D potential field radius
	dmp.setSearchRadius(Vec2f(0.5, 0.5)); // Set the valid search region around given path
	dmp.setMap(map_util, start_pos); // Set map util for collision checking, must be called before planning
	bool valid_dist = dmp.computePath(start_pos, goal_pos, path_jps); // Compute the path given the jps path
	const auto path_dist = dmp.getRawPath();
	if(valid_dist)
	    printf("DMP Path Distance: %f\n", total_distance2f(path_dist));
	else
	    printf("Could not find a valid DMP path\n");

	std::cout << (path_dist.size() == 0? "Path is empty\n" : "Path is not empty. Path size = ") << path_dist.size() << std::endl;

	for(int i = 0; i < path_dist.size(); i++){
		std::cout << path_dist[i][0] << " " << path_dist[i][1] << std::endl;
	}
}