#pragma once

#include <Eigen/Dense>

#include "forces.hpp"
#include "tire_model.hpp"
#include "vehicle_model.hpp"

#include <iostream>

using namespace Eigen;
typedef Matrix<double, 16, 1> State;
typedef Matrix<double, 5, 1>  Control;
typedef Matrix<double, 4, 2>  Slips;
// typedef Matrix<double, 25, 1> Forces;
typedef Matrix<double, 4, 1> Vector4d;

class VehicleSimulator {
  public:
    VehicleSimulator() {
        state_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        control_ << 0.0, 0.0, 0.0, 0.0, 0.0;
        forces_ = Forces();
        slips_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        vehicle_model_.load();
        tire_model_.load();
    };

    const State getState() const {
        return state_;
    };

    void setState(const State state) {
        state_ = state;
    };

    const Control getControl() const {
        return control_;
    };

    void setControl(const Control control) {
        control_ = control;
    };

    const Forces getForces() const {
        return forces_;
    };

    void setForces(const Forces forces) {
        forces_ = forces;
    };

    const Slips getSlips() const {
        return slips_;
    };

    void setSlips(const Slips slips) {
        slips_ = slips;
    };

    VehicleModel getVehicleModel() const {
        return vehicle_model_;
    };

    void setVehicleModel(const VehicleModel& vehicle_model) {
        vehicle_model_ = vehicle_model;
    };

    TireModel getTireModel() const {
        return tire_model_;
    };

    void setTireModel(const TireModel& tire_model) {
        tire_model_ = tire_model;
    };

    State computeStateGradient() const;

    void simulateOneStep(double time_step);

    /**
     * \brief compute the longitudinal slip ratio and lateral slip angle for each wheel
     *
     * \return: a 4x2 Slips matrix where the first column corresponds to the longitudinal slip ratio and the second
     * column to a lateral slip angle
     */
    Slips computeSlips() const;

    Forces computeForces() const;

  private:
    /**
     * \brief compute the tire force using the combined Pacejka tire model
     *
     * \param[out]: Vector4d of the longitudinal tire forces computed in the tire (pneumatic) frame
     * \param[out]:
     * \param[in]:
     * \param[in]:
     * \param[in]:
     * \param[in]:
     *
     */
    void computeTireForcesInTireFrame(Vector4d&      forces_xp,
                                      Vector4d&      forces_yp,
                                      const Slips&   slips,
                                      const Vector4d forces_z,
                                      const Vector4d gamma,
                                      const double   mu) const;

    void convertTireForcesToVehicleFrame(Vector4d&      forces_x,
                                         Vector4d&      forces_y,
                                         const Vector4d forces_xp,
                                         const Vector4d forces_yp,
                                         const Vector4d forces_z) const;

    State computeDynamics(const Vector4d forces_x,
                          const Vector4d forces_y,
                          const Vector4d forces_xp,
                          const Vector4d forces_delta_susp_z) const;

    template <typename T>
    int sign(const T val) const {
        return (T(0) <= val) - (val < T(0));
    };

    State        state_;
    Control      control_;
    Forces       forces_;
    Slips        slips_;
    VehicleModel vehicle_model_;
    TireModel    tire_model_;
};
