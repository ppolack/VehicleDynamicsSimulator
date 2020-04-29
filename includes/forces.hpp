#pragma once

using namespace Eigen;
typedef Matrix<double, 4, 1> Vector4d;

class Forces {
  public:
    Forces() {
        x << 0.0, 0.0, 0.0, 0.0;
        y << 0.0, 0.0, 0.0, 0.0;
        z << 0.0, 0.0, 0.0, 0.0;
        xp << 0.0, 0.0, 0.0, 0.0;
        yp << 0.0, 0.0, 0.0, 0.0;
        suspension << 0.0, 0.0, 0.0, 0.0;
        drag = 0.0;
    };

    Vector4d x;
    Vector4d y;
    Vector4d z;
    Vector4d xp;
    Vector4d yp;
    Vector4d suspension;
    double   drag;
};
