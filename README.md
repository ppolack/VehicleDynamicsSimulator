# VehicleDynamicsSimulator
This repository contains the vehicle simulator that I have made and used in many of my publications. For more details about the model, please refer to my PhD thesis: http://www.theses.fr/2018PSLEM025

## Install Eigen 
sudo apt-get install libeigen3-dev

## To compile from the root repository without cmake
`g++ -o build/test_out vehicle_simulator/src/simulator.cpp vehicle_simulator/src/vehicle_simulator.cpp  $(pkg-config --cflags eigen3) -I./vehicle_simulator/includes --std=c++17`

## To compile with cmake
`cd build`
`cmake ../vehicle_simulator`
`cmake --build .`           
