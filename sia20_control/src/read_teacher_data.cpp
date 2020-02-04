#include <iostream>
#include <sstream>
#include <numeric>
#include <vector>
#include <fanda/Csv.hpp>

int main(int argc, char* argv[])
{
	std::vector<int> file_number = {3, 9, 10, 11, 12, 14, 15, 17, 19, 20, 22, 23, 24, 26, 27, 30};
	std::vector<std::string> file_paths;
	for (auto&& number : file_number){
		std::stringstream ss;
		ss << "/home/harumo/catkin_ws/src/sia20/sia20_control/log/log_" << number << ".csv";
		file_paths.push_back(ss.str());
		std::cout << ss.str() << std::endl;
	}

	
	return 0;
}
