#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/integrate/integrate.hpp>
#include <math.h>

#include "tire_model.hpp"
#include "utils_geom.hpp"
#include "vehicle_model.hpp"
#include "vehicle_simulator.hpp"

#include <boost/bind.hpp>
#include <fstream>
#include <functional>
#include <iostream>
#include <vector>

#include <typeinfo>

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;

class Simulator {
  public:
    std::vector<double> convertToVector(const State state) const {
        std::vector<double> state_vector(state.data(), state.data() + state.rows() * state.cols());
        return state_vector;
    }

    State convertToState(const std::vector<double> state_vect) const {
        State state;
        int   count = 0;

        for (auto it = state_vect.begin(); it != state_vect.end(); ++it) {
            state(count) = *it;
            count++;
        }
        return state;
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
        vehicle_simulator_.setState(convertToState(state_vector));
        simulator_history_.push_back(vehicle_simulator_);
        // rk4_.do_step(
        //     [this](state_type& x, state_type& dxdt, double t) { system(x, dxdt, t); }, state_vector, tt, time_step);
    };

    void exportTrajInCsv() {
        // file pointer
        std::fstream fout;
        std::string  export_file_name;

        export_file_name = "export/traj_test.csv";

        // opens an existing csv file or creates a new file.
        fout.open(export_file_name, std::ios::out);  // std::ios::out | std::ios::app);

        std::cout << "\n" << std::endl;
        std::cout << "\n" << std::endl;
        std::cout << "----------------" << std::endl;
        std::cout << "----------------" << std::endl;
        std::cout << "Exporting trajectory into " << export_file_name << std::endl;

        // Header
        fout << "x_world,"
             << "vx_veh,"
             << "y_world,"
             << "vy_veh,"
             << "z_world,"
             << "vz_veh,"
             << "roll,"
             << "d_roll,"
             << "pitch,"
             << "d_pitch,"
             << "yaw,"
             << "d_yaw,"
             << "omega_fl,"
             << "omega_fr,"
             << "omega_rl,"
             << "omega_rr,"
             << "torque_fl,"
             << "torque_fr,"
             << "torque_rl,"
             << "torque_rr,"
             << "steering,"
             << "tau_x_fl,"
             << "tau_x_fr,"
             << "tau_x_rl,"
             << "tau_x_rr,"
             << "alpha_y_fl,"
             << "alpha_y_fr,"
             << "alpha_y_rl,"
             << "alpha_y_rr,"
             << "force_xp_fl,"
             << "force_xp_fr,"
             << "force_xp_rl,"
             << "force_xp_rr,"
             << "force_yp_fl,"
             << "force_yp_fr,"
             << "force_yp_rl,"
             << "force_yp_rr,"
             << "force_z_fl,"
             << "force_z_fr,"
             << "force_z_rl,"
             << "force_z_rr"
             << "\n";

        // Read the input
        for (auto it = simulator_history_.begin(); it < simulator_history_.end(); ++it) {
            // Insert the data to file
            // std::cout << "print" << (*it).getState()(0, 0) << std::endl;
            fout << (*it).getState()(0, 0) << "," << (*it).getState()(1, 0) << "," << (*it).getState()(2, 0) << ","
                 << (*it).getState()(3, 0) << ", " << (*it).getState()(4, 0) << ", " << (*it).getState()(5, 0) << ", "
                 << (*it).getState()(6, 0) << ", " << (*it).getState()(7, 0) << ", " << (*it).getState()(8, 0) << ", "
                 << (*it).getState()(9, 0) << ", " << (*it).getState()(10, 0) << ", " << (*it).getState()(11, 0) << ", "
                 << (*it).getState()(12, 0) << ", " << (*it).getState()(13, 0) << ", " << (*it).getState()(14, 0)
                 << ", " << (*it).getState()(15, 0) << ", " << (*it).getControl()(0, 0) << ", "
                 << (*it).getControl()(1, 0) << ", " << (*it).getControl()(2, 0) << ", " << (*it).getControl()(3, 0)
                 << ", " << (*it).getControl()(4, 0) << ", " << (*it).getSlips()(0, 0) << ", " << (*it).getSlips()(1, 0)
                 << ", " << (*it).getSlips()(2, 0) << ", " << (*it).getSlips()(3, 0) << ", " << (*it).getSlips()(0, 1)
                 << ", " << (*it).getSlips()(1, 1) << ", " << (*it).getSlips()(2, 1) << ", " << (*it).getSlips()(3, 1)
                 << ", " << (*it).getForces().xp(0, 0) << ", " << (*it).getForces().xp(1, 0) << ", "
                 << (*it).getForces().xp(2, 0) << ", " << (*it).getForces().xp(3, 0) << ", "
                 << (*it).getForces().yp(0, 0) << ", " << (*it).getForces().yp(1, 0) << ", "
                 << (*it).getForces().yp(2, 0) << ", " << (*it).getForces().yp(3, 0) << ", "
                 << (*it).getForces().z(0, 0) << ", " << (*it).getForces().z(1, 0) << ", " << (*it).getForces().z(2, 0)
                 << ", " << (*it).getForces().z(3, 0) << "\n";
        }
    };

  private:
    runge_kutta4<state_type> rk4_;

    VehicleSimulator vehicle_simulator_;

    std::vector<VehicleSimulator> simulator_history_;
};

/**
 * Main function to be called
 */
int main() {
    // fetch simulation params
    double time_end  = 100;
    double time_step = 0.001;  // 0.01 better

    double n_steps = floor(time_end / time_step) + 1;

    Simulator sim = Simulator();

    State  initial_state;
    double vx_start = 0.2;
    double wh_start = vx_start / sim.getVehicleSimulator().getVehicleModel().r_eff;
    initial_state << 0.0, vx_start, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, wh_start, wh_start, wh_start,
        wh_start;

    auto i = 1;

    Control control;
    Control control_init;
    control_init << 100.0, 100.0, 0, 0, 0.1;
    double max_speed = 30;

    std::cout << "Running main" << std::endl;

    VehicleSimulator veh_sim = VehicleSimulator();
    veh_sim                  = sim.getVehicleSimulator();
    veh_sim.setState(initial_state);

    sim.setVehicleSimulator(veh_sim);

    std::cout << "Start simulation" << std::endl;

    for (double tt = time_step; tt <= time_end; tt = tt + time_step) {
        veh_sim = sim.getVehicleSimulator();

        // define the control value
        if (veh_sim.getState()(1) > max_speed) {
            control(0) = 0.0;
            control(1) = 0.0;
        } else {
            control = control_init;
        }
        veh_sim.setControl(control);

        // compute slips and forces
        Slips slips = veh_sim.computeSlips();
        veh_sim.setSlips(slips);
        Forces forces = veh_sim.computeForces();
        veh_sim.setForces(forces);

        // get new state
        sim.setVehicleSimulator(veh_sim);

        sim.simulate(tt, time_step);

        if (fabs(std::remainder(tt, 1)) < time_step / 2) {
            std::cout << "----------------" << std::endl;
            std::cout << "time: " << tt << std::endl;
            std::cout << "X: " << sim.getVehicleSimulator().getState()(0, 0) << std::endl;
            std::cout << "Y: " << sim.getVehicleSimulator().getState()(2, 0) << std::endl;
            std::cout << "yaw: " << normalizeAngle(sim.getVehicleSimulator().getState()(10, 0)) << std::endl;
            std::cout << "speed X: " << sim.getVehicleSimulator().getState()(1, 0) << std::endl;
            std::cout << "tau_x: " << sim.getVehicleSimulator().getSlips()(0, 0) << ", "
                      << sim.getVehicleSimulator().getSlips()(1, 0) << ", "
                      << sim.getVehicleSimulator().getSlips()(2, 0) << ", "
                      << sim.getVehicleSimulator().getSlips()(3, 0) << std::endl;
            std::cout << "alpha_y: " << sim.getVehicleSimulator().getSlips()(0, 1) << ", "
                      << sim.getVehicleSimulator().getSlips()(1, 1) << ", "
                      << sim.getVehicleSimulator().getSlips()(2, 1) << ", "
                      << sim.getVehicleSimulator().getSlips()(3, 1) << std::endl;
            std::cout << "Forces_x: " << sim.getVehicleSimulator().getForces().x[0] << ", "
                      << sim.getVehicleSimulator().getForces().x[1] << ", "
                      << sim.getVehicleSimulator().getForces().x[2] << ", "
                      << sim.getVehicleSimulator().getForces().x[3] << std::endl;
            std::cout << "Forces_y: " << sim.getVehicleSimulator().getForces().y[0] << ", "
                      << sim.getVehicleSimulator().getForces().y[1] << ", "
                      << sim.getVehicleSimulator().getForces().y[2] << ", "
                      << sim.getVehicleSimulator().getForces().y[3] << std::endl;
            std::cout << "Forces_z: " << sim.getVehicleSimulator().getForces().z[0] << ", "
                      << sim.getVehicleSimulator().getForces().z[1] << ", "
                      << sim.getVehicleSimulator().getForces().z[2] << ", "
                      << sim.getVehicleSimulator().getForces().z[3] << std::endl;
        }
        //     // display state
        //     std::cout << "X position: " << sim.getVehicleSimulator().getState()[0] << std::endl;
        //     std::cout << "Y position: " << sim.getVehicleSimulator().getState()[2] << std::endl;
        //     std::cout << "yaw position: " << sim.getVehicleSimulator().getState()[6] << std::endl;
    }

    sim.exportTrajInCsv();

    return 0;
}
