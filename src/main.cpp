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
    std::cout << vehicle_simulator.getState() << std::endl;

    return 0;
}
