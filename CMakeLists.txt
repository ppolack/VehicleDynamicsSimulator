cmake_minimum_required(VERSION 3.5.1)

# set the project name
project(10DoFVehicleSimulator)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)

include_directories(includes ${EIGEN3_INCLUDE_DIR})

# add the executable
add_executable(${CMAKE_PROJECT_NAME} src/vehicle_simulator.cpp src/simulator.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME} yaml-cpp)


# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

## Compile as C++17
add_compile_options(-std=c++17 -Wall -Wextra -Werror)
