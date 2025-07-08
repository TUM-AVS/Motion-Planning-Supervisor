#include "TrajectorySample.hpp"

TrajectorySample::TrajectorySample(CartesianSample m_cartesianSample, CurviLinearSample m_curvilinearSample, size_t m_size, size_t m_actualSize, double m_dT)
: m_cartesianSample(m_cartesianSample), m_curvilinearSample(m_curvilinearSample), m_size(m_size), m_actualSize(m_actualSize), m_dT(m_dT) {}

void TrajectorySample::addCostValueToList(std::string costFunctionName, double cost, double costWeighted)
{
    m_cost += costWeighted;
    m_costMap[costFunctionName] = std::make_pair(cost, costWeighted);
}


void TrajectorySample::addFeasabilityValueToList(std::string feasabilityFunctionsName, double value)
{
    if(value) m_feasible = false;
    m_feasabilityMap[feasabilityFunctionsName] = value;
}

size_t TrajectorySample::size()
{
    return m_size;
}

