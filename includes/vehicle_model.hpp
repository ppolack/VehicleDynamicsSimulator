#pragma once

#include <Eigen/Dense>
typedef Eigen::Matrix<double, 4, 1> Vector4d;

const double GRAVITY = ;

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
        
        mass_suspended = mass_total - 4 * mass_wheel;
        k_susp_r       = std::pow(lf / lr, 2) * k_susp_f;
        d_susp_r       = std::pow(lf / lr, 2) * d_susp_f;

        Vector4d lever_arm_matrix;
        lever_arm_matrix << lr, lr, lf, lf;
        f_susp_0 = mass_suspended * GRAVITY / (2 * (lf + lr)) * lever_arm_matrix;
        fz_0     = f_susp_0 + MatrixXf::Constant(4, 1, mass_wheel * GRAVITY);
    };
};
