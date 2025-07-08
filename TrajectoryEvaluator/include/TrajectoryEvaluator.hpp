#pragma once

#include "TrajectorySample.hpp"
#include "TrajectoryStrategy.hpp"
#include <memory>
#include <vector>
#include <map>

/**
 * @class TrajectoryEvaluator
 * @brief A class to calcualte the kinematic constraints and cost of a trajectory.
 *
 * The trajectory is evaluated by iterating over and executing the stored feasibility and cost functions.
 */
class TrajectoryEvaluator {
public:
    explicit TrajectoryEvaluator();
    
    void evaluateTrajectory(TrajectorySample& trajectory);

private:
    
    std::vector<std::unique_ptr<TrajectoryStrategy>> functions_;
    static const std::map<std::string, double> cost_weights_;
    static const std::map<std::string, double> vehicle_params_;
    static const double desired_velocity_;
};
