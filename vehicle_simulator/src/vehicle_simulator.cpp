#include "vehicle_simulator.hpp"

#include <cmath>
#include <math.h>

const double EPSILON = 0.01;

State VehicleSimulator::computeStateGradient() const {
    Forces forces = VehicleSimulator::computeForces();

    State state = VehicleSimulator::computeDynamics(
        forces.x,
        forces.y,
        forces.xp,
        forces.suspension);  // Warning, should be delta_susp_z normally, not suspension... ???
    return state;
};

void VehicleSimulator::simulateOneStep(double time_step){};

Slips VehicleSimulator::computeSlips() const {
    Slips slips_out;
    slips_out << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    double aaa = vehicle_model_.lw * state_[11];
    double bbb = vehicle_model_.lf * state_[11];
    double ccc = vehicle_model_.lr * state_[11];

    Matrix<double, 4, 2> v_tire;
    v_tire << state_[1] - aaa, state_[3] + bbb, state_[1] + aaa, state_[3] + bbb, state_[1] - aaa, state_[3] - ccc,
        state_[1] + aaa, state_[3] - ccc;

    Matrix<double, 4, 1> v_tire_in_tire_frame;
    v_tire_in_tire_frame << std::cos(control_[4]) * v_tire(0, 0) - std::sin(control_[4]) * v_tire(0, 1),
        std::cos(control_[4]) * v_tire(1, 0) - std::sin(control_[4]) * v_tire(1, 1), v_tire(2, 0), v_tire(3, 0);

    for (int k = 0; k < 4; ++k) {
        // compute longitudinal slip ratio
        if (state_[12 + k] * v_tire_in_tire_frame(k, 0) >= 0) {
            if (abs(vehicle_model_.r_eff * state_[k + 12]) >= abs(v_tire_in_tire_frame(k, 0))) {
                // Throttle
                slips_out(k, 0) = (vehicle_model_.r_eff * state_[k + 12] - v_tire_in_tire_frame(k, 0)) /
                                  (vehicle_model_.r_eff * state_[k + 12] +
                                   sign(state_[k + 12]) * EPSILON);  // check if copysign(,0) is positive
            } else {
                // braking
                slips_out(k, 0) = (vehicle_model_.r_eff * state_[k + 12] - v_tire_in_tire_frame(k, 0)) /
                                  (v_tire_in_tire_frame(k, 0) +
                                   sign(v_tire_in_tire_frame(k, 0)) * EPSILON);  // check if copysign(,0) is positive
            }
        } else if (v_tire_in_tire_frame(k, 0) < 0) {
            // Vx<0 & omega>0
            slips_out(k, 0) = 1;
        } else {
            // Vx>0 & omega<0
            slips_out(k, 0) = -1;
        }

        // compute lateral slip angle
        if (k < 2) {
            // front wheels
            slips_out(k, 1) = control_[4] - atan2(v_tire(k, 1), v_tire(k, 0) + EPSILON * sign(v_tire(k, 0)));
        } else {
            // rear wheels
            slips_out(k, 1) = -atan2(v_tire(k, 1), v_tire(k, 0) + EPSILON * sign(v_tire(k, 0)));
        }
    }

    return slips_out;
}

Forces VehicleSimulator::computeForces() const {
    // compute suspension forces

    Vector4d delta_suspension_z(0, 0, 0, 0);
    Vector4d delta_suspension_forces(0, 0, 0, 0);
    Vector4d suspension_forces(0, 0, 0, 0);

    // compute suspension forces
    for (int k = 0; k < 4; ++k) {
        if (k < 2) {
            // front suspension
            delta_suspension_forces[k] = std::pow(-1, k) * vehicle_model_.lw / 2 * std::sin(state_[6]) -
                                         vehicle_model_.lf * std::cos(state_[6]) * std::sin(state_[8]);
            delta_suspension_z[k] = std::pow(-1, k) * vehicle_model_.lw / 2 * state_[7] * std::cos(state_[6]) +
                                    vehicle_model_.lf * state_[7] * std::sin(state_[6]) * std::sin(state_[8]) -
                                    vehicle_model_.lf * state_[9] * std::cos(state_[6]) * std::cos(state_[8]);
            suspension_forces[k] =
                -vehicle_model_.k_susp_f * delta_suspension_forces[k] - vehicle_model_.d_susp_f * delta_suspension_z[k];
        } else {
            // rear suspension
            delta_suspension_forces[k] = std::pow(-1, k) * vehicle_model_.lw / 2 * std::sin(state_[6]) +
                                         vehicle_model_.lr * std::cos(state_[6]) * std::sin(state_[8]);
            delta_suspension_z[k] = std::pow(-1, k) * vehicle_model_.lw / 2 * state_[7] * std::cos(state_[6]) -
                                    vehicle_model_.lr * state_[7] * std::sin(state_[6]) * std::sin(state_[8]) +
                                    vehicle_model_.lr * state_[9] * std::cos(state_[6]) * std::cos(state_[8]);
            suspension_forces[k] =
                -vehicle_model_.k_susp_r * delta_suspension_forces[k] - vehicle_model_.d_susp_r * delta_suspension_z[k];
        }
    }

    // Normal Forces with load transfer
    Vector4d forces_z;
    forces_z << 0.0, 0.0, 0.0, 0.0;
    forces_z = vehicle_model_.fz_0 + delta_suspension_forces;

    // Drag forces
    double forces_drag = vehicle_model_.drag_coef * std::pow(state_[1], 2);

    Vector4d camber_angles;
    camber_angles << 0.0, 0.0, 0.0, 0.0;

    double mu = 1;

    Slips slips = VehicleSimulator::computeSlips();

    Vector4d forces_xp(0.0, 0.0, 0.0, 0.0);
    Vector4d forces_yp(0.0, 0.0, 0.0, 0.0);

    VehicleSimulator::computeTireForcesInTireFrame(forces_xp, forces_yp, slips, forces_z, camber_angles, mu);

    Vector4d forces_x(0.0, 0.0, 0.0, 0.0);
    Vector4d forces_y(0.0, 0.0, 0.0, 0.0);

    VehicleSimulator::convertTireForcesToVehicleFrame(forces_x, forces_y, forces_xp, forces_yp, forces_z);

    Forces all_forces;
    all_forces.x          = forces_x;
    all_forces.y          = forces_y;
    all_forces.z          = forces_z;
    all_forces.xp         = forces_xp;
    all_forces.yp         = forces_yp;
    all_forces.suspension = suspension_forces;
    all_forces.drag       = forces_drag;

    return all_forces;
}

void VehicleSimulator::computeTireForcesInTireFrame(Vector4d&      forces_xp,
                                                    Vector4d&      forces_yp,
                                                    const Slips&   slips,
                                                    const Vector4d forces_z,
                                                    const Vector4d gamma,
                                                    const double   mu) const {
    Vector4d tau_x;
    tau_x << 0.0, 0.0, 0.0, 0.0;
    Vector4d slip_angle;
    slip_angle << 0.0, 0.0, 0.0, 0.0;
    for (int k = 0; k < 4; ++k) {
        tau_x[k]      = slips(k, 0);
        slip_angle[k] = slips(k, 1);
    }

    Vector4d tau_shift;
    tau_shift << 0.0, 0.0, 0.0, 0.0;
    Vector4d slip_angle_shift;
    slip_angle_shift << 0.0, 0.0, 0.0, 0.0;
    Vector4d Fxp0;
    Fxp0 << 0.0, 0.0, 0.0, 0.0;
    Vector4d Fyp0;
    Fyp0 << 0.0, 0.0, 0.0, 0.0;
    Vector4d Fxp;
    Fxp << 0.0, 0.0, 0.0, 0.0;
    Vector4d Fyp;
    Fyp << 0.0, 0.0, 0.0, 0.0;
    Vector4d G_xa;
    G_xa << 0.0, 0.0, 0.0, 0.0;
    Vector4d G_yk;
    G_yk << 0.0, 0.0, 0.0, 0.0;

    for (int k = 0; k < 4; ++k) {
        double dFz = (forces_z[k] - vehicle_model_.fz_0[k]) / (vehicle_model_.fz_0[k]);

        // PURE SLIP MODELS

        // Longitudinal forces (pure longitudinal slip)
        double S_Hx  = tire_model_.p_Hx1;
        tau_shift[k] = tau_x[k] + S_Hx;

        double C_x  = tire_model_.p_Cx1;
        double mu_x = mu * (tire_model_.p_Dx1 + tire_model_.p_Dx2 * dFz);
        double D_x  = mu_x * forces_z[k];
        double E_x  = (tire_model_.p_Ex1 + tire_model_.p_Ex2 * dFz + tire_model_.p_Ex3 * std::pow(dFz, 2)) *
                     (1 - tire_model_.p_Ex4 * sign(tau_shift[k]));
        double K_xk = forces_z[k] * (tire_model_.p_Kx1 + tire_model_.p_Kx2 * dFz) * exp(tire_model_.p_Kx3 * dFz);
        double B_x  = K_xk / (C_x * D_x);
        double S_Vx = forces_z[k] * (tire_model_.p_Vx1 + tire_model_.p_Vx2 * dFz);

        Fxp0[k] =
            D_x * std::sin(C_x * atan(B_x * tau_shift[k] - E_x * (B_x * tau_shift[k] - atan(B_x * tau_shift[k])))) +
            S_Vx;

        // Lateral forces (pure lateral slip)
        double S_Hy         = (tire_model_.p_Hy1 + tire_model_.p_Hy2 * dFz) + tire_model_.p_Hy3 * gamma[k];
        slip_angle_shift[k] = slip_angle[k] + S_Hy;

        double C_y = tire_model_.p_Cy1;
        double mu_y =
            mu * (tire_model_.p_Dy1 + tire_model_.p_Dy2 * dFz) * (1 - tire_model_.p_Dy3 * std::pow(gamma[k], 2));
        double D_y = mu_y * forces_z[k];
        double E_y = (tire_model_.p_Ey1 + tire_model_.p_Ey2 * dFz) *
                     (1 - (tire_model_.p_Ey3 + tire_model_.p_Ey4 * gamma[k] * sign(slip_angle_shift[k])));
        double K_ya0 = tire_model_.p_Ky1 * vehicle_model_.fz_0[k] *
                       std::sin(2 * atan(forces_z[k] / (tire_model_.p_Ky2 * vehicle_model_.fz_0[k])));
        double K_ya = K_ya0 * (1 - tire_model_.p_Ky3 * std::pow(gamma[k], 2));
        double B_y  = K_ya / (C_y * D_y);
        double S_Vy = forces_z[k] * ((tire_model_.p_Vy1 + tire_model_.p_Vy2 * dFz) +
                                     (tire_model_.p_Vy3 + tire_model_.p_Vy4 * dFz) * gamma[k]);

        Fyp0[k] = D_y * std::sin(C_y * atan(B_y * slip_angle_shift[k] -
                                            E_y * (B_y * slip_angle_shift[k] - atan(B_y * slip_angle_shift[k])))) +
                  S_Vy;

        // COMBINED SLIP MODELS

        // Longitudinal forces (combined slip)
        double B_xa  = tire_model_.r_Bx1 * std::cos(atan(tire_model_.r_Bx2 * tau_x[k]));
        double C_xa  = tire_model_.r_Cx1;
        double E_xa  = tire_model_.r_Ex1 + tire_model_.r_Ex2 * dFz;
        double S_Hxa = tire_model_.r_Hx1;

        slip_angle_shift[k] = slip_angle[k] + S_Hxa;

        G_xa[k] = (std::cos(C_xa * atan(B_xa * slip_angle_shift[k] -
                                        E_xa * (B_xa * slip_angle_shift[k] - atan(B_xa * slip_angle_shift[k]))))) /
                  (std::cos(C_xa * atan(B_xa * S_Hxa - E_xa * (B_xa * S_Hxa - atan(B_xa * S_Hxa)))));

        Fxp[k] = G_xa[k] * Fxp0[k];

        // Lateral forces (combined slip)
        double B_yk  = tire_model_.r_By1 * std::cos(atan(tire_model_.r_By2 * (slip_angle[k] - tire_model_.r_By3)));
        double C_yk  = tire_model_.r_Cy1;
        double E_yk  = tire_model_.r_Ey1 + tire_model_.r_Ey2 * dFz;  // pour l'instant, sans variation de masse;
        double S_Hyk = tire_model_.r_Hy1 + tire_model_.r_Hy2 * dFz;  // pour l'instant, sans variation de masse;
        double D_Vyk = mu_y * forces_z[k] *
                       (tire_model_.r_Vy1 + tire_model_.r_Vy2 * dFz + tire_model_.r_Vy3 * gamma[k]) *
                       std::cos(atan(tire_model_.r_Vy4 * slip_angle[k]));
        double S_Vyk = D_Vyk * std::sin(tire_model_.r_Vy5 * atan(tire_model_.r_Vy6 * tau_x[k]));

        tau_shift[k] = tau_x[k] + S_Hyk;
        G_yk[k] =
            (std::cos(C_yk * atan(B_yk * tau_shift[k] - E_yk * (B_yk * tau_shift[k] - atan(B_yk * tau_shift[k]))))) /
            (std::cos(C_yk * atan(B_yk * S_Hyk - E_yk * (B_yk * S_Hyk - atan(B_yk * S_Hyk)))));
        Fyp[k] = G_yk[k] * Fyp0[k] + S_Vyk;

        forces_xp = Fxp;
        forces_yp = Fyp;
    }
}

void VehicleSimulator::convertTireForcesToVehicleFrame(Vector4d&      forces_x,
                                                       Vector4d&      forces_y,
                                                       const Vector4d forces_xp,
                                                       const Vector4d forces_yp,
                                                       const Vector4d forces_z) const {
    Vector4d delta_steer;
    delta_steer << control_[4], control_[4], 0.0, 0.0;  // to be improved with ackerman steering

    forces_x << 0.0, 0.0, 0.0, 0.0;
    forces_y << 0.0, 0.0, 0.0, 0.0;

    for (int k = 0; k < 4; ++k) {
        // Full model
        forces_x[k] =
            (forces_xp[k] * std::cos(delta_steer[k]) - forces_yp[k] * std::sin(delta_steer[k])) * std::cos(state_[8]) -
            forces_z[k] * std::sin(state_[8]);
        forces_y[k] =
            (forces_xp[k] * std::cos(delta_steer[k]) - forces_yp[k] * std::sin(delta_steer[k])) * std::sin(state_[6]) *
                std::sin(state_[8]) +
            (forces_yp[k] * std::cos(delta_steer[k]) + forces_xp[k] * std::sin(delta_steer[k])) * std::cos(state_[6]) -
            forces_z[k] * std::sin(state_[6]) * std::cos(state_[8]);
    }
}

State VehicleSimulator::computeDynamics(const Vector4d forces_x,
                                        const Vector4d forces_y,
                                        const Vector4d forces_xp,
                                        const Vector4d forces_delta_susp_z) const {
    State state_gradient;

    // Translation Motions
    // X
    state_gradient[0] = state_[1] * std::cos(state_[10]) - state_[3] * std::sin(state_[10]);
    state_gradient[1] =
        state_[11] * state_[3] - state_[9] * state_[5] +
        forces_x.sum() / vehicle_model_.mass_total;  //à Ajouter ???  - F_aero*cos(y(9))  + gravity*sin(y(9)-slope);
    // Y
    state_gradient[2] = state_[1] * std::sin(state_[10]) + state_[3] * std::cos(state_[10]);
    state_gradient[3] =
        -state_[11] * state_[1] + state_[7] * state_[5] +
        forces_y.sum() /
            vehicle_model_.mass_total;  // à ajouter ??? - gravity*sin(y(7)-road_bank_angle)*cos(y(9)-slope);
    // Z
    state_gradient[4] = state_[5];
    state_gradient[5] = 0.0;  // 1./vehicle_model_.mass_susp*(F_susp(1)+F_susp(2)+F_susp(3)+F_susp(4)) -
                              // gravity*cos(y(7))*cos(y(9));

    // Rotation Motions
    //// roll
    state_gradient[6] = state_[7];
    state_gradient[7] =
        1.0 / vehicle_model_.inertia_x *
        (vehicle_model_.lw / 2.0 *
             (forces_delta_susp_z[0] + forces_delta_susp_z[2] - forces_delta_susp_z[1] - forces_delta_susp_z[3]) +
         state_[4] * forces_y.sum());
    //// pitch
    state_gradient[8] = state_[9];
    state_gradient[9] =
        1 / vehicle_model_.inertia_y *
        (-vehicle_model_.lf * (forces_delta_susp_z[0] + forces_delta_susp_z[1]) +
         vehicle_model_.lr * (forces_delta_susp_z[2] + forces_delta_susp_z[3]) - state_[4] * forces_x.sum());
    //// yaw
    state_gradient[10] = state_[11];
    state_gradient[11] =
        1. / vehicle_model_.inertia_z *
        (vehicle_model_.lf * (forces_y[0] + forces_y[1]) - vehicle_model_.lr * (forces_y[2] + forces_y[3]) +
         vehicle_model_.lw / 2.0 * (forces_x[1] + forces_x[3] - forces_x[0] - forces_x[2]));  // + Czi ??? quest-ce ?

    //// Wheel rotations
    //// omega
    state_gradient[12] =
        (control_[0] - vehicle_model_.r_eff * forces_xp[0]) / vehicle_model_.inertia_r;  // Front left tire
    state_gradient[13] =
        (control_[1] - vehicle_model_.r_eff * forces_xp[1]) / vehicle_model_.inertia_r;  // Front right tire
    state_gradient[14] =
        (control_[2] - vehicle_model_.r_eff * forces_xp[2]) / vehicle_model_.inertia_r;  // Rear left tire
    state_gradient[15] =
        (control_[3] - vehicle_model_.r_eff * forces_xp[3]) / vehicle_model_.inertia_r;  // Rear right tire

    return state_gradient;
};
