cmake_minimum_required(VERSION 3.5.1)

# set the project name
project(simulator10dof)

include_directories(includes)

# add the executable
add_executable(10dofVehicleSim src/main.cpp src/vehicle_simulator.cpp)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
