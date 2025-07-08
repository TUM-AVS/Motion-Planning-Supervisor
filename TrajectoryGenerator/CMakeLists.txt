cmake_minimum_required(VERSION 3.16)
project(trajectory_node LANGUAGES C CXX)

set(GENERATOR_DATA "${CMAKE_CURRENT_SOURCE_DIR}/data")

if(NOT TARGET CycloneDDS::ddsc)
  find_package(CycloneDDS REQUIRED)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

idlc_generate(TARGET TrajectorySampleData_lib FILES "${CMAKE_CURRENT_SOURCE_DIR}/msg/TrajectorySampleData.idl" WARNINGS no-implicit-extensibility)
idlc_generate(TARGET CartesianSampleData_lib FILES "${CMAKE_CURRENT_SOURCE_DIR}/msg/CartesianSampleData.idl" WARNINGS no-implicit-extensibility)
idlc_generate(TARGET CurvilinearSampleData_lib FILES "${CMAKE_CURRENT_SOURCE_DIR}/msg/CurvilinearSampleData.idl" WARNINGS no-implicit-extensibility)
idlc_generate(TARGET ResultData_lib FILES "${CMAKE_CURRENT_SOURCE_DIR}/msg/ResultData.idl" WARNINGS no-implicit-extensibility)

add_executable(Trajectory_node
  ${CMAKE_CURRENT_SOURCE_DIR}/src/trajectory_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/csv_parser.cpp
)


target_link_libraries(Trajectory_node PRIVATE
    CycloneDDS::ddsc
    CurvilinearSampleData_lib
    CartesianSampleData_lib
    TrajectorySampleData_lib
    ResultData_lib
)
