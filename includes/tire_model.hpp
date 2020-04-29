#pragma once

class TireModel {
  public:
    // Pure longitudinal slip parameters for Longitudinal Forces
    double p_Cx1;
    double p_Dx1;
    double p_Dx2;
    double p_Ex1;
    double p_Ex2;
    double p_Ex3;
    double p_Ex4;
    double p_Kx1;
    double p_Kx2;
    double p_Kx3;
    double p_Hx1;
    double p_Hx2;
    double p_Vx1;
    double p_Vx2;

    // Pure side slip parameters for Lateral Forces
    double p_Cy1;
    double p_Dy1;
    double p_Dy2;
    double p_Dy3;
    double p_Ey1;
    double p_Ey2;
    double p_Ey3;
    double p_Ey4;
    double p_Ky1;
    double p_Ky2;
    double p_Ky3;
    double p_Hy1;
    double p_Hy2;
    double p_Hy3;
    double p_Vy1;
    double p_Vy2;
    double p_Vy3;
    double p_Vy4;

    // Pure side slip parameters for Self-aligning Moment
    double q_Bz1;
    double q_Bz2;
    double q_Bz3;
    double q_Bz4;
    double q_Bz5;
    double q_Bz10;
    double q_Cz1;
    double q_Dz1;
    double q_Dz2;
    double q_Dz3;
    double q_Dz4;
    double q_Dz6;
    double q_Dz7;
    double q_Dz8;
    double q_Dz9;
    double q_Ez1;
    double q_Ez2;
    double q_Ez3;
    double q_Ez4;
    double q_Ez5;
    double q_Hz1;
    double q_Hz2;
    double q_Hz3;
    double q_Hz4;

    // Longitudinal parameters for combined slip
    double r_Bx1;
    double r_Bx2;
    double r_Cx1;
    double r_Ex1;  // ??? Not given in Pacejka's book
    double r_Ex2;  // ??? Not given in Pacejka's book
    double r_Hx1;

    // Lateral parameters for combined slip
    double r_By1;
    double r_By2;
    double r_By3;
    double r_Cy1;
    double r_Ey1;  //??? Not given in Pacejka's book
    double r_Ey2;  //??? Not given in Pacejka's book
    double r_Hy1;  // 0.02; Normalement, 0.02 sauf si on veut que Fx^2+ Fy^2 soit parfaitement symétrique
    double r_Hy2;  //??? Not given in Pacejka's book
    double r_Vy1;
    double r_Vy2;
    double r_Vy3;
    double r_Vy4;
    double r_Vy5;
    double r_Vy6;

    // Self-aligning Moment parameters for combined slip
    double s_sz1;
    double s_sz2;
    double s_sz3;
    double s_sz4;

    void load(){};
};
