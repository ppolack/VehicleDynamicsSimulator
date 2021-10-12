import argparse
import csv
import pandas as pd
import matplotlib.pyplot as plt


def main():

    parser = argparse.ArgumentParser(description='Process data.')
    parser.add_argument('filename', type=str,
                        help='path to the trajectory file to plot')

    args = parser.parse_args()

    # xdata = []
    # ydata = []
    # thetadata = []

    # print(args.filename)

    # with open(args.filename) as csvfile:
    #     trajfilereader = csv.reader(csvfile, delimiter=',', quotechar='|')
    #     for row in trajfilereader:
    #         xdata.append(row[0])
    #         ydata.append(row[1])
    #         thetadata.append(row[2])

    # d = {'X': xdata, 'Y': ydata, 'theta': thetadata}
    # df = pd.DataFrame(data=d)

    print(args.filename)

    df2 = pd.read_csv(args.filename, sep=',')
    print(df2.head())

# "x_world", "vx_veh", "y_world", "vy_veh", "z_world", "vz_veh", "roll", "d_roll", "pitch", "d_pitch", "yaw", "d_yaw", "omega_fl", "omega_fr", "omega_rl", "omega_rr", "torque_fl", "torque_fr", "torque_rl", "torque_rr", "tau_x_fl", "tau_x_fr", "tau_x_rl", "tau_x_rr", "alpha_y_fl", "alpha_y_fr", "alpha_y_rl", "alpha_y_rr, "

    for col in df2.columns:
        print(col)

    plt.figure(0)
    plt.plot(df2['x_world'], df2['y_world'], 'xb')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Path')
    plt.axis('equal')

    plt.figure(1)
    ax1 = plt.subplot(311)
    plt.plot(df2['x_world'])
    plt.ylabel('X (m)')
    ax2 = plt.subplot(312, sharex=ax1)
    plt.plot(df2['y_world'])
    plt.ylabel('Y (m)')
    ax3 = plt.subplot(313, sharex=ax1)
    plt.plot(df2['yaw'])
    plt.ylabel('yaw (rad)')

    plt.figure(2)
    ax1 = plt.subplot(311)
    plt.plot(df2['vx_veh'])
    plt.ylabel('vx (m/s)')
    ax2 = plt.subplot(312, sharex=ax1)
    plt.plot(df2['vy_veh'])
    plt.ylabel('vy (m/s)')
    ax3 = plt.subplot(313, sharex=ax1)
    plt.plot(df2['d_yaw'])
    plt.ylabel('dyaw (rad/s)')

    plt.figure(3)
    ax1 = plt.subplot(311)
    plt.plot(df2['roll'])
    plt.ylabel('roll (rad)')
    ax2 = plt.subplot(312, sharex=ax1)
    plt.plot(df2['pitch'])
    plt.ylabel('pitch (rad)')
    ax3 = plt.subplot(313, sharex=ax1)
    plt.plot(df2['yaw'])
    plt.ylabel('yaw (rad)')

    plt.figure(4)
    plt.plot(df2['omega_fl'], 'r', label="fl")
    plt.plot(df2['omega_fr'], 'b', label="fr")
    plt.plot(df2['omega_rl'], 'g', label="rl")
    plt.plot(df2['omega_rr'], 'c', label="rr")
    plt.ylabel('omega (rad/s)')
    plt.legend()

    plt.figure(5)
    plt.plot(df2['torque_fl'], 'r', label="fl")
    plt.plot(df2['torque_fr'], 'b', label="fr")
    plt.plot(df2['torque_rl'], 'g', label="rl")
    plt.plot(df2['torque_rr'], 'c', label="rr")
    plt.ylabel('torque (N.m.)')
    plt.legend()

    plt.figure(6)
    ax1 = plt.subplot(211)
    plt.plot(df2['tau_x_fl'], 'r', label="fl")
    plt.plot(df2['tau_x_fr'], 'b', label="fr")
    plt.plot(df2['tau_x_rl'], 'g', label="rl")
    plt.plot(df2['tau_x_rr'], 'c', label="rr")
    plt.ylabel('slip ratio (-)')
    plt.legend()

    ax2 = plt.subplot(212, sharex=ax1)
    plt.plot(df2['alpha_y_fl'], 'r', label="fl")
    plt.plot(df2['alpha_y_fr'], 'b', label="fr")
    plt.plot(df2['alpha_y_rl'], 'g', label="rl")
    plt.plot(df2['alpha_y_rr'], 'c', label="rr")
    plt.ylabel('slip angle (rad)')
    plt.legend()

    plt.figure(7)
    ax1 = plt.subplot(311)
    plt.plot(df2['d_roll'])
    plt.ylabel('droll (rad/s)')
    ax2 = plt.subplot(312, sharex=ax1)
    plt.plot(df2['d_pitch'])
    plt.ylabel('dpitch (rad/s)')
    ax3 = plt.subplot(313, sharex=ax1)
    plt.plot(df2['d_yaw'])
    plt.ylabel('dyaw (rad/s)')

    plt.figure(8)
    plt.title("forces generated in the tire frame")
    ax1 = plt.subplot(311)
    plt.plot(df2['force_xp_fl'], 'r', label="fl")
    plt.plot(df2['force_xp_fr'], 'b', label="fr")
    plt.plot(df2['force_xp_rl'], 'g', label="rl")
    plt.plot(df2['force_xp_rr'], 'c', label="rr")
    plt.ylabel('Forces_{xp} (N)')
    plt.legend()

    ax2 = plt.subplot(312, sharex=ax1)
    plt.plot(df2['force_yp_fl'], 'r', label="fl")
    plt.plot(df2['force_yp_fr'], 'b', label="fr")
    plt.plot(df2['force_yp_rl'], 'g', label="rl")
    plt.plot(df2['force_yp_rr'], 'c', label="rr")
    plt.ylabel('Forces_{yp} (N)')

    ax3 = plt.subplot(313, sharex=ax1)
    plt.plot(df2['force_z_fl'], 'r', label="fl")
    plt.plot(df2['force_z_fr'], 'b', label="fr")
    plt.plot(df2['force_z_rl'], 'g', label="rl")
    plt.plot(df2['force_z_rr'], 'c', label="rr")
    plt.ylabel('Forces_z (N)')

    plt.legend()

    # plt.figure(99)
    # plt.subplot(221)
    # # equivalent but more general
    # ax1 = plt.subplot(2, 2, 1)

    # # add a subplot with no frame
    # ax2 = plt.subplot(222, frameon=False)

    # # add a polar subplot
    # plt.subplot(223, projection='polar')

    # # add a red subplot that shares the x-axis with ax1
    # plt.subplot(224, sharex=ax1, facecolor='red')

    # # delete ax2 from the figure
    # plt.delaxes(ax2)

    # # add ax2 to the figure again
    # plt.subplot(ax2)

    # plt.figure(2)
    # plt.polar(df2['x_world'], df2['y_world'])
    plt.show()


if __name__ == "__main__":
    main()
    pass
