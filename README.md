# 10 DoF Vehicle Dynamics Simulator

This project contains the code of the 10 degrees of freedom vehicle simulator that I developed during my PhD Thesis at Mines ParisTech, and used in many of my publications.

For full explanation of the model, please refer to the chapter 2 of my [PhD thesis](http://www.theses.fr/2018PSLEM025)

Examples of video obtained coupling this simulator to the rendering environment PreScan developed by TNO. In the first video, you can see the vehicle is loosing control due to slip.

[![Watch the video](https://img.youtube.com/vi/BRpmdIxTz-0/hqdefault.jpg)](https://www.youtube.com/watch?v=BRpmdIxTz-0)

- [High speed trajectory planning](https://www.youtube.com/watch?v=BRpmdIxTz-0&feature=youtu.be)

- [Rain Condition (road coefficient $\mu$ set to 0.7)](https://www.youtube.com/watch?v=6LFNhpcmssY)

- [Snow Condition (road coefficient $\mu$ set to 0.2)](https://www.youtube.com/watch?v=qUT5sFY_RE4)

- [Deep Learning for coupled longitudinal and lateral control](https://www.youtube.com/watch?v=yyWy1uavlXs)

Known Limitations:

- missing Ackermann steering wheel (both front wheels have the same angle)

Credits:
Please quote my PhD if using my simulator for publication or whatever other use you may have of it.

## Installation

Install dependencies

```bash
sudo apt install libeigen3-dev
sudo apt-get install libboost-all-dev
```

Build project

```bash
mkdir build
cd build
cmake ..
make
```

## Usage

The full trajectory obtained is then exported to a csv file defined by `export_file_name` containing all information:

| Value | Description |
| :--------------- |:---------------|
| x_world | X coordinate in the world frame (meter) |
| vx_veh | longitudinal speed of the vehicle in vehicle frame (meter/s) |
| y_world | Y coordinate in the world frame (meter) |
| vy_veh | lateral speed of the vehicle in vehicle frame (meter/s) |
| z_world | Z coordinate in the world frame (meter) |
| vz_veh | vertical speed of the vehicle in vehicle frame (meter/s) |
| roll | roll angle of the carbody (rad) |
| d_roll | roll rate of the carbody (rad/s) |
| pitch | pitch angle of the carbody (rad) |
| d_pitch | pitch rate of the carbody (rad/s) |
| yaw | yaw angle of the carbody (rad) |
| d_yaw | yaw rate of the carbody (rad/s) |
| omega_fl | front left wheel rotation speed (rad/s) |
| omega_fr | front right wheel rotation speed (rad/s) |
| omega_rl | rear left wheel rotation speed (rad/s) |
| omega_rr | rear right wheel rotation speed (rad/s) |
| torque_fl | front left wheel applied torque (Nm) |
| torque_fr | front right wheel applied torque (Nm) |
| torque_rl | rear left wheel applied torque (Nm) |
| torque_rr | rear right wheel applied torque (Nm) |
| steering | steering angle of both front wheels (rad) |
| tau_x_fl | front left wheel longitudinal slip ratio (-) |
| tau_x_fr | front right wheel longitudinal slip ratio (-) |
| tau_x_rl | rear left wheel longitudinal slip ratio (-) |
| tau_x_rr | rear right wheel longitudinal slip ratio (-) |
| alpha_y_fl | front left wheel lateral slip angle (rad) |
| alpha_y_fr | front right wheel lateral slip angle (rad) |
| alpha_y_rl | rear left wheel lateral slip angle (rad) |
| alpha_y_rr | rear right wheel lateral slip angle (rad) |
| force_xp_fl | front left wheel longitudinal force in tire frame (N) |
| force_xp_fr | front right wheel longitudinal force in tire frame (N) |
| force_xp_rl | rear left wheel longitudinal force in tire frame (N) |
| force_xp_rr | rear right wheel longitudinal force in tire frame (N) |
| force_yp_fl | front left wheel lateral force in tire frame (N) |
| force_yp_fr | front right wheel lateral force in tire frame (N) |
| force_yp_rl | rear left wheel lateral force in tire frame (N) |
| force_yp_rr | rear right wheel lateral force in tire frame (N) |
| force_z_fl | front left wheel normal reaction force (N) |
| force_z_fr | front right wheel normal reaction force (N) |
| force_z_rl | rear left wheel normal reaction force (N) |
| force_z_rr | rear right wheel normal reaction force (N) |

## License

[GNU GENERAL PUBLIC LICENSE](https://www.gnu.org/licenses/gpl-3.0.html)
