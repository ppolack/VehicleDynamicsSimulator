#pragma once
#include <yaml-cpp/yaml.h>

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
      YAML::Node tire_config_file = YAML::LoadFile("config/tire_description.yaml");

      // Pure longitudinal slip parameters for computing longitudinal forces
      p_Cx1 = tire_config_file["p_Cx1"].as<double>();
      p_Dx1 = tire_config_file["p_Dx1"].as<double>();
      p_Dx2 = tire_config_file["p_Dx2"].as<double>();
      p_Ex1 = tire_config_file["p_Ex1"].as<double>();
      p_Ex2 = tire_config_file["p_Ex2"].as<double>();
      p_Ex3 = tire_config_file["p_Ex3"].as<double>();
      p_Ex4 = tire_config_file["p_Ex4"].as<double>();
      p_Kx1 = tire_config_file["p_Kx1"].as<double>();
      p_Kx2 = tire_config_file["p_Kx2"].as<double>();
      p_Kx3 = tire_config_file["p_Kx3"].as<double>();
      p_Hx1 = tire_config_file["p_Hx1"].as<double>();
      p_Hx2 = tire_config_file["p_Hx2"].as<double>();
      p_Vx1 = tire_config_file["p_Vx1"].as<double>();
      p_Vx2 = tire_config_file["p_Vx2"].as<double>();

      // Pure side slip parameters for computing lateral forces
      p_Cy1 = tire_config_file["p_Cy1"].as<double>();
      p_Dy1 = tire_config_file["p_Dy1"].as<double>();
      p_Dy2 = tire_config_file["p_Dy2"].as<double>();
      p_Dy3 = tire_config_file["p_Dy3"].as<double>();
      p_Ey1 = tire_config_file["p_Ey1"].as<double>();
      p_Ey2 = tire_config_file["p_Ey2"].as<double>();
      p_Ey3 = tire_config_file["p_Ey3"].as<double>();
      p_Ey4 = tire_config_file["p_Ey4"].as<double>();
      p_Ky1 = tire_config_file["p_Ky1"].as<double>();
      p_Ky2 = tire_config_file["p_Ky2"].as<double>();
      p_Ky3 = tire_config_file["p_Ky3"].as<double>();
      p_Hy1 = tire_config_file["p_Hy1"].as<double>();
      p_Hy2 = tire_config_file["p_Hy2"].as<double>();
      p_Hy3 = tire_config_file["p_Hy3"].as<double>();
      p_Vy1 = tire_config_file["p_Vy1"].as<double>();
      p_Vy2 = tire_config_file["p_Vy2"].as<double>();
      p_Vy3 = tire_config_file["p_Vy3"].as<double>();
      p_Vy4 = tire_config_file["p_Vy4"].as<double>();

      // Pure side slip parameters for computing self-aligning moment
      q_Bz1  = tire_config_file["q_Bz1"].as<double>();
      q_Bz2  = tire_config_file["q_Bz2"].as<double>();
      q_Bz3  = tire_config_file["q_Bz3"].as<double>();
      q_Bz4  = tire_config_file["q_Bz4"].as<double>();
      q_Bz5  = tire_config_file["q_Bz5"].as<double>();
      q_Bz10 = tire_config_file["q_Bz10"].as<double>();
      q_Cz1  = tire_config_file["q_Cz1"].as<double>();
      q_Dz1  = tire_config_file["q_Dz1"].as<double>();
      q_Dz2  = tire_config_file["q_Dz2"].as<double>();
      q_Dz3  = tire_config_file["q_Dz3"].as<double>();
      q_Dz4  = tire_config_file["q_Dz4"].as<double>();
      q_Dz6  = tire_config_file["q_Dz6"].as<double>();
      q_Dz7  = tire_config_file["q_Dz7"].as<double>();
      q_Dz8  = tire_config_file["q_Dz8"].as<double>();
      q_Dz9  = tire_config_file["q_Dz9"].as<double>();
      q_Ez1  = tire_config_file["q_Ez1"].as<double>();
      q_Ez2  = tire_config_file["q_Ez2"].as<double>();
      q_Ez3  = tire_config_file["q_Ez3"].as<double>();
      q_Ez4  = tire_config_file["q_Ez4"].as<double>();
      q_Ez5  = tire_config_file["q_Ez5"].as<double>();
      q_Hz1  = tire_config_file["q_Hz1"].as<double>();
      q_Hz2  = tire_config_file["q_Hz2"].as<double>();
      q_Hz3  = tire_config_file["q_Hz3"].as<double>();
      q_Hz4  = tire_config_file["q_Hz4"].as<double>();

      // Longitudinal parameters for combined slip
      r_Bx1 = tire_config_file["r_Bx1"].as<double>();
      r_Bx2 = tire_config_file["r_Bx2"].as<double>();
      r_Cx1 = tire_config_file["r_Cx1"].as<double>();
      r_Ex1 = tire_config_file["r_Ex1"].as<double>();  // Not given in Pacejka's book
      r_Ex2 = tire_config_file["r_Ex2"].as<double>();  // Not given in Pacejka's book
      r_Hx1 = tire_config_file["r_Hx1"].as<double>();

      // Lateral parameters for combined slip
      r_By1 = tire_config_file["r_By1"].as<double>();
      r_By2 = tire_config_file["r_By2"].as<double>();
      r_By3 = tire_config_file["r_By3"].as<double>();
      r_Cy1 = tire_config_file["r_Cy1"].as<double>();
      r_Ey1 = tire_config_file["r_Ey1"].as<double>();  // Not given in Pacejka's book
      r_Ey2 = tire_config_file["r_Ey2"].as<double>();  // Not given in Pacejka's book
      r_Hy1 = tire_config_file["r_Hy1"].as<double>();  // 0.02 norammly, except if one wants Fx^2+ Fy^2 to be perfectly symmetric
      r_Hy2 = tire_config_file["r_Hy2"].as<double>();  // Not given in Pacejka's book
      r_Vy1 = tire_config_file["r_Vy1"].as<double>();
      r_Vy2 = tire_config_file["r_Vy2"].as<double>();
      r_Vy3 = tire_config_file["r_Vy3"].as<double>();
      r_Vy4 = tire_config_file["r_Vy4"].as<double>();
      r_Vy5 = tire_config_file["r_Vy5"].as<double>();
      r_Vy6 = tire_config_file["r_Vy6"].as<double>();

      // Self-aligning moment parameters for combined slip
      s_sz1 = tire_config_file["s_sz1"].as<double>();
      s_sz2 = tire_config_file["s_sz2"].as<double>();
      s_sz3 = tire_config_file["s_sz3"].as<double>();
      s_sz4 = tire_config_file["s_sz4"].as<double>();
    };
};
