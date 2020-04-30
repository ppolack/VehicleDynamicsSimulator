#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/integrate/integrate.hpp>
#include <math.h>

#include "tire_model.hpp"
#include "vehicle_model.hpp"
#include "vehicle_simulator.hpp"

#include <boost/bind.hpp>
#include <functional>
#include <iostream>

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;

class Simulator {
  public:
    std::vector<double> convertToVector(const State state) const {
        std::vector<double> state_vector(state.data(), state.data() + state.rows() * state.cols());
        return state_vector;
    }
    void observer(const state_type& x, const double t) {
        std::cout << t << "   " << x[0] << "   " << x[2] << "   " << x[10] << std::endl;
    }

    // the system function can be a classical functions
    void system(state_type& x, state_type& dxdt, double t) {
        dxdt = convertToVector(
            vehicle_simulator_.computeStateGradient());  // probleme -> ne prend pas en compte les valeurs à 0
    }

    VehicleSimulator getVehicleSimulator() {
        return vehicle_simulator_;
    };

    void setVehicleSimulator(VehicleSimulator vehicle_sim) {
        vehicle_simulator_ = vehicle_sim;
    };

    void simulate(double tt, double time_step) {
        state_type state_vector;
        state_vector = convertToVector(vehicle_simulator_.getState());
        rk4_.do_step(
            std::bind(&Simulator::system, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
            state_vector,
            tt,
            time_step);
        // rk4_.do_step(
        //     [this](state_type& x, state_type& dxdt, double t) { system(x, dxdt, t); }, state_vector, tt, time_step);
    };

  private:
    runge_kutta4<state_type> rk4_;

    VehicleSimulator vehicle_simulator_;
};

/**
 * Main function to be called
 */
int main() {
    // fetch simulation params
    double time_end  = 1;
    double time_step = 0.1;  // 0.01 better

    double n_steps = floor(time_end / time_step) + 1;

    Simulator sim = Simulator();

    State  initial_state;
    double vx_start = 0.2;
    double wh_start = vx_start / sim.getVehicleSimulator().getVehicleModel().r_eff;
    initial_state << 0.0, vx_start, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, wh_start, wh_start, wh_start,
        wh_start;

    auto i = 1;

    Control control;
    control << 100.0, 100.0, 0, 0, 0.01;

    std::cout << "Running main" << std::endl;

    VehicleSimulator veh_sim = VehicleSimulator();
    veh_sim                  = sim.getVehicleSimulator();
    veh_sim.setState(initial_state);

    sim.setVehicleSimulator(veh_sim);

    std::cout << "Start simulation" << std::endl;

    for (double tt = 0; tt <= time_end; tt = tt + time_step) {
        veh_sim = sim.getVehicleSimulator();

        // define the control value
        veh_sim.setControl(control);

        // compute slips and forces
        Slips slips = veh_sim.computeSlips();
        veh_sim.setSlips(slips);
        Forces forces = veh_sim.computeForces();
        veh_sim.setForces(forces);

        // get new state
        sim.setVehicleSimulator(veh_sim);

        sim.simulate(tt, time_step);

        std::cout << "----------------" << std::endl;
        std::cout << "time: " << tt << std::endl;
        std::cout << "X: " << sim.getVehicleSimulator().getState()(0, 0) << std::endl;
        std::cout << "Y: " << sim.getVehicleSimulator().getState()(2, 0) << std::endl;
        std::cout << "yaw: " << sim.getVehicleSimulator().getState()(10, 0) << std::endl;
        std::cout << "tau_x: " << sim.getVehicleSimulator().getSlips()(0, 0) << ", "
                  << sim.getVehicleSimulator().getSlips()(1, 0) << ", " << sim.getVehicleSimulator().getSlips()(2, 0)
                  << ", " << sim.getVehicleSimulator().getSlips()(3, 0) << std::endl;
        std::cout << "alpha_y: " << sim.getVehicleSimulator().getSlips()(0, 1) << ", "
                  << sim.getVehicleSimulator().getSlips()(1, 1) << ", " << sim.getVehicleSimulator().getSlips()(2, 1)
                  << ", " << sim.getVehicleSimulator().getSlips()(3, 1) << std::endl;
        std::cout << "Forces_x: " << sim.getVehicleSimulator().getForces().x[0] << ", "
                  << sim.getVehicleSimulator().getForces().x[1] << ", " << sim.getVehicleSimulator().getForces().x[2]
                  << ", " << sim.getVehicleSimulator().getForces().x[3] << std::endl;
        std::cout << "Forces_y: " << sim.getVehicleSimulator().getForces().y[0] << ", "
                  << sim.getVehicleSimulator().getForces().y[1] << ", " << sim.getVehicleSimulator().getForces().y[2]
                  << ", " << sim.getVehicleSimulator().getForces().y[3] << std::endl;
        std::cout << "Forces_z: " << sim.getVehicleSimulator().getForces().z[0] << ", "
                  << sim.getVehicleSimulator().getForces().z[1] << ", " << sim.getVehicleSimulator().getForces().z[2]
                  << ", " << sim.getVehicleSimulator().getForces().z[3] << std::endl;

        //     // display state
        //     std::cout << "X position: " << sim.getVehicleSimulator().getState()[0] << std::endl;
        //     std::cout << "Y position: " << sim.getVehicleSimulator().getState()[2] << std::endl;
        //     std::cout << "yaw position: " << sim.getVehicleSimulator().getState()[6] << std::endl;
    }

    return 0;
}
