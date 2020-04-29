#include <boost/numeric/odeint/integrate/integrate.hpp>
#include <math.h>

#include "tire_model.hpp"
#include "vehicle_model.hpp"
#include "vehicle_simulator.hpp"

/**
 * Main function to be called
 */
int main() {
    // fetch simulation params
    double time_end  = 10.0;
    double time_step = 0.01;

    double n_steps = floor(time_end / time_step) + 1;

    VehicleSimulator vehicle_simulator = VehicleSimulator();

    std::cout << "Running main" << std::endl;

    State  initial_state;
    double vx_start = 0.2;
    double wh_start = vx_start / vehicle_simulator.getVehicleModel().r_eff;
    initial_state << 0.0, vx_start, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, wh_start, wh_start, wh_start,
        wh_start;

    Control control;
    control << 100.0, 100.0, 0, 0, 0.02;

    vehicle_simulator.setState(initial_state);

    std::cout << "Start simulation" << std::endl;

    for (int tt = 0; tt <= time_end; tt = tt + time_step) {
        // define the control value
        vehicle_simulator.setControl(control);

        // compute slips and forces
        Slips slips = vehicle_simulator.computeSlips();
        vehicle_simulator.setSlips(slips);
        Forces forces = vehicle_simulator.computeForces();
        vehicle_simulator.setForces(forces);

        // get new state
        // State state = integration...;
        // vehicle_simulator.setState(state);

        std::cout << "X position: " << vehicle_simulator.getState()[0] << std::endl;
        std::cout << "Y position: " << vehicle_simulator.getState()[2] << std::endl;
        std::cout << "yaw position: " << vehicle_simulator.getState()[6] << std::endl;
    }

    return 0;
}
