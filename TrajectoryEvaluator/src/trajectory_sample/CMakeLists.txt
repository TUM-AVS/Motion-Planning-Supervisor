
add_library(trajectory_sample STATIC
  src/CartesianSample.cpp
  src/CurvilinearSample.cpp
  src/TrajectorySample.cpp
)

target_include_directories(trajectory_sample PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include  
)

target_include_directories(trajectory_sample PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})
