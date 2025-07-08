#include "TrajectoryEvaluator.hpp"

#include "CheckAccelerationConstraint.hpp"
#include "CheckCurvatureConstraints.hpp"
#include "CheckCurvatureRateConstrains.hpp"
#include "CheckYawRateConstraint.hpp"

#include "CalculateAccelerationCost.hpp"
#include "CalculateDistanceToReferencePathCost.hpp"
#include "CalculateJerkCost.hpp"
#include "CalculateLateralAccelerationCost.hpp"
#include "CalculateLateralVelocityCost.hpp"
#include "CalculateLongitudinalAccelerationCost.hpp"
#include "CalculateLongitudinalVelocityCost.hpp"
#include "CalculateNegativeAccelerationCost.hpp"
#include "CalculateNegativeOrientationOffsetCost.hpp"
#include "CalculateNegativeVelocityOffsetCost.hpp"
#include "CalculateOrientationOffsetCost.hpp"
#include "CalculatePositiveAccelerationCost.hpp"
#include "CalculatePositiveOrientationOffsetCost.hpp"
#include "CalculatePositiveVelocityOffsetCost.hpp"
#include "CalculateVelocityOffsetCost.hpp"

#include "TrajectorySample.hpp"
#include "TrajectoryStrategy.hpp"

#include <string>

static constexpr size_t NUM_FUNCTIONS = 19;


const std::map<std::string, double> TrajectoryEvaluator::cost_weights_ = {
    {"acceleration", 0.0},
    {"jerk", 0.0},
    {"lateral_jerk", 0.2},
    {"longitudinal_jerk", 0.2},
    {"orientation_offset", 0.0},
    {"path_length", 0.0},
    {"lane_center_offset", 0.0},
    {"velocity_offset", 1.0},
    {"velocity", 0.0},
    {"distance_to_reference_path", 5.0},
    {"responsibility", 0}
};

const std::map<std::string, double> TrajectoryEvaluator::vehicle_params_ = {
    {"length", 4.508},
    {"width", 1.61},
    {"mass", 1093.3},
    {"wheelbase", 2.59},
    {"delta_max", 1.066},
    {"delta_min", -1.066},
    {"a_max", 11.5},
    {"v_max", 50.8},
    {"v_switch", 7.319},
    {"v_delta_max", 0.4},
    {"v_delta_min", -0.4},
    {"wb_front_axle", 1.156},
    {"wb_rear_axle", 1.422},
    {"cr_vehicle_id", 2} 
};

const double TrajectoryEvaluator::desired_velocity_ = 7.0;

/*
@brief Stores the selected cost and feasibility functions
*/
TrajectoryEvaluator::TrajectoryEvaluator() {
    functions_.reserve(NUM_FUNCTIONS);
    
    functions_.emplace_back(std::make_unique<CheckAccelerationConstraint>(vehicle_params_.at("v_switch"), vehicle_params_.at("a_max"), true));
    functions_.emplace_back(std::make_unique<CheckCurvatureConstraint>(vehicle_params_.at("delta_max"), vehicle_params_.at("wheelbase"), true));
    functions_.emplace_back(std::make_unique<CheckCurvatureRateConstraint>(vehicle_params_.at("wheelbase"), vehicle_params_.at("v_delta_max"), true));
    functions_.emplace_back(std::make_unique<CheckYawRateConstraint>(vehicle_params_.at("delta_max"), vehicle_params_.at("wheelbase"), true));
    
    functions_.emplace_back(std::make_unique<CalculateAccelerationCost>("CalculateAccelerationCost", cost_weights_.at("acceleration")));
    functions_.emplace_back(std::make_unique<CalculateDistanceToReferencePathCost>("CalculateDistanceToReferencePathCost", cost_weights_.at("distance_to_reference_path")));
    functions_.emplace_back(std::make_unique<CalculateJerkCost>("CalculateJerkCost", cost_weights_.at("jerk")));
    functions_.emplace_back(std::make_unique<CalculateLateralAccelerationCost>("CalculateLateralAccelerationCost", cost_weights_.at("lateral_jerk")));
    functions_.emplace_back(std::make_unique<CalculateLateralVelocityCost>("CalculateLateralVelocityCost", cost_weights_.at("velocity")));
    functions_.emplace_back(std::make_unique<CalculateLongitudinalAccelerationCost>("CalculateLongitudinalAccelerationCost", cost_weights_.at("longitudinal_jerk")));
    functions_.emplace_back(std::make_unique<CalculateLongitudinalVelocityCost>("CalculateLongitudinalVelocityCost", cost_weights_.at("velocity")));
    functions_.emplace_back(std::make_unique<CalculateNegativeAccelerationCost>("CalculateNegativeAccelerationCost", cost_weights_.at("acceleration")));
    functions_.emplace_back(std::make_unique<CalculateNegativeOrientationOffsetCost>("CalculateNegativeOrientationOffsetCost", cost_weights_.at("orientation_offset")));
    functions_.emplace_back(std::make_unique<CalculateNegativeVelocityOffsetCost>("CalculateNegativeVelocityOffsetCost", cost_weights_.at("velocity_offset"), desired_velocity_));
    functions_.emplace_back(std::make_unique<CalculateOrientationOffsetCost>("CalculateOrientationOffsetCost", cost_weights_.at("orientation_offset")));
    functions_.emplace_back(std::make_unique<CalculatePositiveAccelerationCost>("CalculatePositiveAccelerationCost", cost_weights_.at("acceleration")));
    functions_.emplace_back(std::make_unique<CalculatePositiveOrientationOffsetCost>("CalculatePositiveOrientationOffsetCost", cost_weights_.at("orientation_offset")));
    functions_.emplace_back(std::make_unique<CalculatePositiveVelocityOffsetCost>("CalculatePositiveVelocityOffsetCost", cost_weights_.at("velocity_offset"), desired_velocity_));
    functions_.emplace_back(std::make_unique<CalculateVelocityOffsetCost>("CalculateVelocityOffsetCost", cost_weights_.at("velocity_offset"), desired_velocity_, 0.1, 1.1, false, 2));    
}

/*
@brief Iterates over all defined cost and feasibility functions and passes the TrajectorySample 

@param trajectory TrajectorySample object representing a trajectory candidate
*/
void TrajectoryEvaluator::evaluateTrajectory(TrajectorySample& trajectory) {
    for (auto& func : functions_) {
        func->evaluateTrajectory(trajectory);
    }

}
