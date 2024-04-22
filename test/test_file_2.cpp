#include <iostream>
#include <vector>
#include <chrono>
#include <dmp_planner_interface.h>

int main(int argc, char **argv)
{

	auto start = std::chrono::high_resolution_clock::now();
	std::vector<std::vector<float>> map = {
		{0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	1,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
		{0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	0,	0,	1,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0},
		{0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0},
		{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	0,	0,	1,	1,	0,	0,	0,	0,	0},
		{0,	0,	0,	0,	0,	0,	1,	0,	1,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0},
		{0,	0,	0,	0,	0,	0,	1,	1,	0,	0,	1,	0,	0,	0,	0,	0,	0,	1,	0,	1,	0,	0,	0,	0,	0},
		{0,	0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	1,	0,	1,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0},
		{0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	1,	1,	0,	0,	1,	0,	0,	1,	0,	0,	0,	0,	0},
		{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0},
		{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
		{0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0}
	};

	auto path = plan({0.5, 0.5}, {5, 1}, {5, 24}, map, 1, true);
	auto end = std::chrono::high_resolution_clock::now();
	std::cout << "Plan Duration: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()*1e-3 << " ms" << std::endl;

	for(int i = 0; i < path.size(); i++){
		std::cout << path[i][0] << " " << path[i][1] << std::endl;
	}

	std::cout << std::endl;
}