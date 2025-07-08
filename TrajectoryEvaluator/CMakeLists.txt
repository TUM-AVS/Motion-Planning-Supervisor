# Copyright (c) 2022-2023, Arm Limited.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22.1)

set(CONF_FILE "prj_actuation.conf")
# Do not overwrite the OVERLAY_CONFIG value provided by the user.
set(OVERLAY_CONFIG "prj_net.conf;${OVERLAY_CONFIG}" CACHE STRING "overlay config" FORCE)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(zephyr_app LANGUAGES C CXX)

include(GNUInstallDirs)

find_package(Eigen3 REQUIRED)


# Build CycloneDDS.
include(cyclonedds_config.cmake)
create_cdds_lib(${CMAKE_CURRENT_SOURCE_DIR}/../cyclonedds)


# Get compile options from Zephyr to be passed to the Autoware packages.
zephyr_get_include_directories_for_lang_as_string(C Cincludes)
zephyr_get_include_directories_for_lang_as_string(CXX CXXincludes)
zephyr_get_system_include_directories_for_lang_as_string(C Csystem_includes)
zephyr_get_system_include_directories_for_lang_as_string(CXX CXXsystem_includes)
zephyr_get_compile_options_for_lang_as_string(C Coptions)
zephyr_get_compile_options_for_lang_as_string(CXX CXXoptions)

# Consistent float abi.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=vfp -mfloat-abi=hard")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfpu=vfp -mfloat-abi=hard")

set(external_project_cflags "${Cincludes} ${Csystem_includes} ${Coptions}") 
set(external_project_cxxflags "${CXXincludes} ${CXXsystem_includes} ${CXXoptions}") 

set(CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/install -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER} -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER} -DCMAKE_C_FLAGS=${external_project_cflags} -DCMAKE_CXX_FLAGS=${external_project_cxxflags})

add_subdirectory(src/cost_functions)
add_subdirectory(src/feasibility_functions)
add_subdirectory(src/trajectory_sample)

target_sources(app PRIVATE
  msg/CartesianSampleData.c
  msg/CurvilinearSampleData.c
  msg/TrajectorySampleData.c
  msg/ResultData.c
  src/communication.cpp
  src/TrajectoryEvaluator.cpp
)

target_include_directories(app PRIVATE
  include
  msg
  ${CMAKE_CURRENT_BINARY_DIR}/install/include
  ${CycloneDDS_INCLUDE_DIR}
)
target_include_directories(app PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})

target_link_directories(app PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/install/lib)

target_link_libraries(app PRIVATE
  cdds_lib
  cost_functions
  feasibility_functions
  trajectory_sample
)
