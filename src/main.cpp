// #include <boost/numeric/odeint.hpp>
// #include <boost/numeric/odeint/integrate/integrate.hpp>
// #include <math.h>

// #include "tire_model.hpp"
// #include "vehicle_model.hpp"
// #include "vehicle_simulator.hpp"

// #include <iostream>

// using namespace boost::numeric::odeint;

// // TEMPORARY
// // Defining a shorthand for the type of the mathematical state
// typedef std::vector<double> state_type;

// // System to be solved: dx/dt = -2 x
// void my_system(const state_type& x, state_type& dxdt, const double t) {
//     dxdt[0] = -2 * x[0];
// }

// // Observer, prints time and state when called (during integration)
// void my_observer(const state_type& x, const double t) {
//     std::cout << t << "   " << x[0] << std::endl;
// }

// std::vector<double> convertToVector(const State state) {
//     // std::vector<double> state_vector;

//     std::vector<double> state_vector(state.data(), state.data() + state.rows() * state.cols());

//     // for (auto itr = state.data(); itr < state.data() + state.size(); ++itr) {
//     //     state_vector.push_back(*itr);
//     // }
//     return state_vector;
// }

// // the system function can be a classical functions
// void system_pp(state_type& x, state_type& dxdt, double t) {
//     VehicleSimulator vehicle_simulator = VehicleSimulator();
//     dxdt                               = convertToVector(
//         vehicle_simulator.computeStateGradient());  // probleme -> ne prend pas en compte les valeurs à 0
// }

// /**
//  * Main function to be called
//  */
// int main() {
//     // fetch simulation params
//     double time_end  = 1;
//     double time_step = 0.1;  // 0.01 better

//     double n_steps = floor(time_end / time_step) + 1;

//     VehicleSimulator vehicle_simulator = VehicleSimulator();

//     std::cout << "Running main" << std::endl;

//     State  initial_state;
//     double vx_start = 0.2;
//     double wh_start = vx_start / vehicle_simulator.getVehicleModel().r_eff;
//     initial_state << 0.0, vx_start, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, wh_start, wh_start, wh_start,
//         wh_start;

//     Control control;
//     control << 100.0, 100.0, 0, 0, 0.01;

//     vehicle_simulator.setState(initial_state);

//     std::cout << "Start simulation" << std::endl;

//     runge_kutta4<state_type> rk4;
//     for (double tt = 0; tt <= time_end; tt = tt + time_step) {
//         // define the control value
//         vehicle_simulator.setControl(control);

//         // compute slips and forces
//         Slips slips = vehicle_simulator.computeSlips();
//         vehicle_simulator.setSlips(slips);
//         Forces forces = vehicle_simulator.computeForces();
//         vehicle_simulator.setForces(forces);

//         // get new state
//         std::vector<double> state_vector = convertToVector(vehicle_simulator.getState());
//         // state_type          x0(1);  // Initial condition, vector of 1 element (scalar problem)
//         // x0[0] = 10.0;

//         // Integration parameters
//         // double t0 = 0.0;
//         // double t1 = 10.0;
//         // double dt = 0.1;

//         rk4.do_step(system_pp, state_vector, tt, time_step);

//         // integrate(convertToVector(vehicle_simulator.computeStateGradient()),
//         //           state_vector,
//         //           tt,
//         //           tt + time_step,
//         //           time_step,
//         //           my_observer);
//         // State state = integration...;
//         // vehicle_simulator.setState(state);

//         std::cout << "----------------" << std::endl;
//         std::cout << "time: " << tt << std::endl;
//         std::cout << "tau_x: " << vehicle_simulator.getSlips()(0, 0) << ", " << vehicle_simulator.getSlips()(1, 0)
//                   << ", " << vehicle_simulator.getSlips()(2, 0) << ", " << vehicle_simulator.getSlips()(3, 0)
//                   << std::endl;
//         std::cout << "alpha_y: " << vehicle_simulator.getSlips()(0, 1) << ", " << vehicle_simulator.getSlips()(1, 1)
//                   << ", " << vehicle_simulator.getSlips()(2, 1) << ", " << vehicle_simulator.getSlips()(3, 1)
//                   << std::endl;
//         std::cout << "Forces_x: " << vehicle_simulator.getForces().x[0] << ", " << vehicle_simulator.getForces().x[1]
//                   << ", " << vehicle_simulator.getForces().x[2] << ", " << vehicle_simulator.getForces().x[3]
//                   << std::endl;
//         std::cout << "Forces_y: " << vehicle_simulator.getForces().y[0] << ", " << vehicle_simulator.getForces().y[1]
//                   << ", " << vehicle_simulator.getForces().y[2] << ", " << vehicle_simulator.getForces().y[3]
//                   << std::endl;
//         std::cout << "Forces_z: " << vehicle_simulator.getForces().z[0] << ", " << vehicle_simulator.getForces().z[1]
//                   << ", " << vehicle_simulator.getForces().z[2] << ", " << vehicle_simulator.getForces().z[3]
//                   << std::endl;

//         //     // display state
//         //     std::cout << "X position: " << vehicle_simulator.getState()[0] << std::endl;
//         //     std::cout << "Y position: " << vehicle_simulator.getState()[2] << std::endl;
//         //     std::cout << "yaw position: " << vehicle_simulator.getState()[6] << std::endl;
//     }

//     return 0;
// }
