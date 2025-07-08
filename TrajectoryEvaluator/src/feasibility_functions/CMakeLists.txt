
add_library(feasibility_functions STATIC
  src/CheckAccelerationConstraint.cpp
  src/CheckCurvatureConstraints.cpp
  src/CheckCurvatureRateConstrains.cpp
  src/CheckYawRateConstraint.cpp
)

target_include_directories(feasibility_functions PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include 
)

target_include_directories(feasibility_functions PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})

target_link_libraries(feasibility_functions PUBLIC trajectory_sample)
