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

    void load() {
        // Pure longitudinal slip parameters for computing longitudinal forces
        p_Cx1 = 1.65;
        p_Dx1 = 1;
        p_Dx2 = 0;
        p_Ex1 = -0.5;
        p_Ex2 = 0;
        p_Ex3 = 0;
        p_Ex4 = 0;
        p_Kx1 = 12;
        p_Kx2 = 10;
        p_Kx3 = -0.6;
        p_Hx1 = 0;
        p_Hx2 = 0;
        p_Vx1 = 0;
        p_Vx2 = 0;

        // Pure side slip parameters for computing lateral forces
        p_Cy1 = 1.3;
        p_Dy1 = 1;
        p_Dy2 = 0;
        p_Dy3 = 0;
        p_Ey1 = -1;
        p_Ey2 = 0;
        p_Ey3 = 0;
        p_Ey4 = 0;
        p_Ky1 = 10;
        p_Ky2 = 1.5;
        p_Ky3 = 0;
        p_Hy1 = 0;
        p_Hy2 = 0;
        p_Hy3 = 0.25;
        p_Vy1 = 0;
        p_Vy2 = 0;
        p_Vy3 = 0.15;
        p_Vy4 = 0;

        // Pure side slip parameters for computing self-aligning moment
        q_Bz1  = 6;
        q_Bz2  = -4;
        q_Bz3  = 0.6;
        q_Bz4  = 0;
        q_Bz5  = 0;
        q_Bz10 = 0.7;
        q_Cz1  = 1.05;
        q_Dz1  = 0.12;
        q_Dz2  = -0.03;
        q_Dz3  = 0;
        q_Dz4  = -1;
        q_Dz6  = 0;
        q_Dz7  = 0;
        q_Dz8  = 0.6;
        q_Dz9  = 0.2;
        q_Ez1  = -10;
        q_Ez2  = 0;
        q_Ez3  = 0;
        q_Ez4  = 0;
        q_Ez5  = 0;
        q_Hz1  = 0;
        q_Hz2  = 0;
        q_Hz3  = 0;
        q_Hz4  = 0;

        // Longitudinal parameters for combined slip
        r_Bx1 = 5;
        r_Bx2 = 8;
        r_Cx1 = 1;
        r_Ex1 = 0;  // Not given in Pacejka's book
        r_Ex2 = 0;  // Not given in Pacejka's book
        r_Hx1 = 0;

        // Lateral parameters for combined slip
        r_By1 = 7;
        r_By2 = 2.5;
        r_By3 = 0;
        r_Cy1 = 1;
        r_Ey1 = 0;  // Not given in Pacejka's book
        r_Ey2 = 0;  // Not given in Pacejka's book
        r_Hy1 = 0;  // 0.02 norammly, except if one wants Fx^2+ Fy^2 to be perfectly symmetric
        r_Hy2 = 0;  // Not given in Pacejka's book
        r_Vy1 = 0;
        r_Vy2 = 0;
        r_Vy3 = -0.2;
        r_Vy4 = 14;
        r_Vy5 = 1.9;
        r_Vy6 = 10;

        // Self-aligning moment parameters for combined slip
        s_sz1 = 0;
        s_sz2 = -0.1;
        s_sz3 = -1.0;
        s_sz4 = 0;
    };
};
