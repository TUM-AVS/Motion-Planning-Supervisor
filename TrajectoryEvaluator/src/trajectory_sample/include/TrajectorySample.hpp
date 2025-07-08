#pragma once

#include <stddef.h>
#include <Eigen/Core>
#include <optional>
#include <string>
#include <unordered_map>
#include <memory>

#include "CartesianSample.hpp"
#include "CurvilinearSample.hpp"
#include "polynomial.hpp"


class TrajectorySample
{
public:
    TrajectorySample(CartesianSample m_cartesianSample, CurviLinearSample m_curvilinearSample, size_t m_size, size_t m_actualSize, double m_dT);


    CartesianSample m_cartesianSample;
    CurviLinearSample m_curvilinearSample;

    std::unordered_map<std::string, std::pair<double,double>> m_costMap;
    std::unordered_map<std::string, double> m_feasabilityMap;

    size_t m_size;
    size_t m_actualSize;
    double m_dT; // = NaN
    double m_cost = 0.0;

    bool m_valid = true;
    bool m_feasible = true;

    void addCostValueToList(std::string costFunctionName, double cost, double costWeighted);

    void addFeasabilityValueToList(std::string costFunctionName, double value);


    size_t size();
};

