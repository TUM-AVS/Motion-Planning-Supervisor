#pragma once

#include "rapidcsv.h"
#include <vector>
#include <map>
#include <string>

/*
@brief Converts a string in format "[x,y,z]" into list of floats [x,y,z].

@param input String to convert
*/
std::vector<double> str_to_float_list(std::string input);


/*
@brief Reads trajectories from a csv file.
*/
std::vector<std::map<std::string, std::vector<double>>> get_data();
