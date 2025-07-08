#include "csv_parser.hpp"
#include "rapidcsv.h"
#include <sstream>

const std::string GENERATOR_DATA = "@GENERATOR_DATA@";

std::vector<double> str_to_float_list(std::string input){
    try{
        input.erase(input.begin());
        input.erase(input.end() - 1);

        std::stringstream stream(input);
        std::string segment;
        std::vector<double> result;

        while(std::getline(stream, segment, ' ')){
            if(!segment.empty()){
                result.push_back(std::stod(segment));
            }   
        }
        return result;
    } catch (const std::exception& e){
        std::cerr << "Failed to parse line from csv: " << e.what() << std::endl; 
        return std::vector<double>();
    }
}

std::vector<std::map<std::string, std::vector<double>>> get_data(){
    try{
        std::string file_path = GENERATOR_DATA + "/trajectories.csv";
        rapidcsv::Document doc(file_path);

        std::vector<std::string> x = doc.GetColumn<std::string>("x");
        std::vector<std::string> y = doc.GetColumn<std::string>("y");
        std::vector<std::string> theta = doc.GetColumn<std::string>("theta");
        std::vector<std::string> v = doc.GetColumn<std::string>("v");
        std::vector<std::string> a = doc.GetColumn<std::string>("a");
        std::vector<std::string> kappa = doc.GetColumn<std::string>("kappa");
        std::vector<std::string> kappa_dot = doc.GetColumn<std::string>("kappa_dot");
        std::vector<std::string> s = doc.GetColumn<std::string>("s");
        std::vector<std::string> d = doc.GetColumn<std::string>("d");
        std::vector<std::string> theta_curv = doc.GetColumn<std::string>("theta_curv");
        std::vector<std::string> d_ddot = doc.GetColumn<std::string>("d_ddot");
        std::vector<std::string> d_dot = doc.GetColumn<std::string>("d_dot");
        std::vector<std::string> s_ddot = doc.GetColumn<std::string>("s_ddot");
        std::vector<std::string> s_dot = doc.GetColumn<std::string>("s_dot");

        std::size_t number_rows = x.size();

        std::vector<std::map<std::string, std::vector<double>>> result;
        
        for (size_t i = 0; i < number_rows; i++) {
            std::map<std::string, std::vector<double>> row{};
            
            row.insert(std::make_pair("x", str_to_float_list(x.at(i))));
            row.insert(std::make_pair("y", str_to_float_list(y.at(i))));
            row.insert(std::make_pair("theta", str_to_float_list(theta.at(i))));
            row.insert(std::make_pair("v", str_to_float_list(v.at(i))));
            row.insert(std::make_pair("a", str_to_float_list(a.at(i))));
            row.insert(std::make_pair("kappa", str_to_float_list(kappa.at(i))));
            row.insert(std::make_pair("kappa_dot", str_to_float_list(kappa_dot.at(i))));
            row.insert(std::make_pair("s", str_to_float_list(s.at(i))));
            row.insert(std::make_pair("d", str_to_float_list(d.at(i))));
            row.insert(std::make_pair("theta_curv", str_to_float_list(theta_curv.at(i))));
            row.insert(std::make_pair("d_ddot", str_to_float_list(d_ddot.at(i))));
            row.insert(std::make_pair("d_dot", str_to_float_list(d_dot.at(i))));
            row.insert(std::make_pair("s_ddot", str_to_float_list(s_ddot.at(i))));
            row.insert(std::make_pair("s_dot", str_to_float_list(s_dot.at(i))));
            
            result.push_back(row);
        }
        
        return result;
    }catch (const std::exception& e){
        std::cerr << "Error processing csv: " << e.what() << std::endl; 
        return std::vector<std::map<std::string, std::vector<double>>>();
    }
}