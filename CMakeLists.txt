cmake_minimum_required(VERSION 3.5.1)

# set the project name
project(10DoFVehicleSimulator)

include_directories(includes)

# add the executable
add_executable(${CMAKE_PROJECT_NAME} src/vehicle_simulator.cpp src/simulator.cpp)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

## Compile as C++17
add_compile_options(-std=c++17 -Wall -Wextra -Werror)
