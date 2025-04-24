#pragma once
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
typedef Eigen::Matrix<double, 4, 1> Vector4d;

const double GRAVITY = 9.81;

class VehicleModel {
  public:
    double lf;
    double lr;
    double lw;
    double r_eff;
    double h_cog;
    double mass_total;
    double mass_suspended;
    double mass_wheel;
    double inertia_x;
    double inertia_y;
    double inertia_z;
    double inertia_r;
    double k_susp_f;  // stiffness factor [Nm] of the front suspension
    double d_susp_f;  // damping factor [Nm/s] of the front suspension
    double k_susp_r;  // stiffness factor [Nm] of the rear suspension
    double d_susp_r;  // damping factor [Nm/s] of the rear suspension
    double front_surface;
    double drag_coef;

    Vector4d f_susp_0;
    Vector4d fz_0;

    void load() {
        YAML::Node vehicle_config_file = YAML::LoadFile("config/vehicle_description.yaml");

        // load values from yaml file
        lf            = vehicle_config_file["lf"].as<double>();
        lr            = vehicle_config_file["lr"].as<double>();
        lw            = vehicle_config_file["lw"].as<double>();
        r_eff         = vehicle_config_file["r_eff"].as<double>();
        h_cog         = vehicle_config_file["h_cog"].as<double>();
        mass_total    = vehicle_config_file["mass_total"].as<double>();
        mass_wheel    = vehicle_config_file["mass_wheel"].as<double>();
        inertia_x     = vehicle_config_file["inertia_x"].as<double>();
        inertia_y     = vehicle_config_file["inertia_y"].as<double>();
        inertia_z     = vehicle_config_file["inertia_z"].as<double>();
        inertia_r     = vehicle_config_file["inertia_r"].as<double>();
        k_susp_f      = vehicle_config_file["k_susp_f"].as<double>();
        d_susp_f      = vehicle_config_file["d_susp_f"].as<double>();
        front_surface = vehicle_config_file["front_surface"].as<double>();
        drag_coef     = vehicle_config_file["drag_coef"].as<double>();

        // compute remaining values
        mass_suspended = mass_total - 4 * mass_wheel;
        k_susp_r       = std::pow(lf / lr, 2) * k_susp_f;
        d_susp_r       = std::pow(lf / lr, 2) * d_susp_f;

        Vector4d lever_arm_matrix;
        lever_arm_matrix << lr, lr, lf, lf;
        f_susp_0 = mass_suspended * GRAVITY / (2 * (lf + lr)) * lever_arm_matrix;

        Eigen::MatrixXd wheel_weight = Eigen::MatrixXd::Constant(4, 1, mass_wheel * GRAVITY);
        fz_0                         = f_susp_0 + wheel_weight;
    };
};
