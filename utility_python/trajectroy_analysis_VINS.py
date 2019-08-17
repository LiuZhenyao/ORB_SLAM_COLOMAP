import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import gps_data_analysis
from numpy.linalg import inv
import quaternion
from scipy.interpolate import RegularGridInterpolator


csv_file_path = "/home/shu/Desktop/SF3000-recordings/2019_07_12_0955_54_perceptionLog_StructSensorSF3000.csv"
# csv_file_path = "/home/shu/Desktop/SF3000-recordings/2019_07_15_2046_57_perceptionLog_StructSensorSF3000.csv"
# csv_file_path = "/home/shu/Desktop/SF3000-recordings/2019_07_15_2047_17_perceptionLog_StructSensorSF3000.csv"
PVT_msg, INS_msg, IMU_msg = gps_data_analysis.read_gps_data(csv_file_path)
timestamps_pvt, easting, northing, altitude, velocityEasting, velocityNorthing, velocityUp = gps_data_analysis.sparse_PVT_msg(PVT_msg)

# Set first measurement of GPS as origin point
timestamps_pvt = np.asarray(timestamps_pvt)[4500:-1]
X = np.asarray([(item - easting[4500]) for item in easting])[4500:-1]
Y = np.asarray([(item - northing[4500]) for item in northing])[4500:-1]
Z = np.asarray([(item - altitude[4500]) for item in altitude])[4500:-1]


# GPS_position = np.zeros((3, len(X)))
# GPS_position[0, :] = np.asmatrix(X)
# GPS_position[1, :] = np.asmatrix(Y)
# GPS_position[2, :] = np.asmatrix(Z)
# GPS_position[3,:] = 1.0

# transformation from mynteye imu to GPS
# Rx_minus30 = np.matrix([[1, 0, 0],
#                        [0, np.cos(-30*pi/180), -np.sin(-30*pi/180)],
#                        [0, np.sin(-30*pi/180), np.cos(-30*pi/180)]])
#
# Ry_minus90 = np.matrix([[0, 0, -1],
#                         [0, 1, 0],
#                         [1, 0, 0]])
#
# Rx_90 = np.matrix([[1, 0, 0],
#                    [0, 0, -1],
#                    [0, 1, 0]])
# R = Rx_90 * Ry_minus90 * Rx_minus30


# Ry_minus60 = np.matrix([[np.cos(-60*pi/180), 0, np.sin(-60*pi/180)],
#                         [0, 1, 0],
#                         [-np.sin(-60*pi/180), 0, np.cos(-60*pi/180)]])
#
# R = Ry_minus60
#
# t = [-0.534, -0.013, 0.500] # [m]
#
# T = np.zeros((4,4), np.float)
# T[0:3, 0:3] = R
# T[0:3, -1] = t
# T[-1,-1] =1
#
# # transformation from GPS to mynteye IMU
# GPS_position_in_mynteye_frame = inv(np.asmatrix(T)) * np.asmatrix(GPS_position)

# fig = plt.figure(2)
# ax = fig.gca(projection='3d')
# ax.plot(GPS_position_in_mynteye_frame[0,:].tolist()[0], GPS_position_in_mynteye_frame[1,:].tolist()[0], GPS_position_in_mynteye_frame[2,:].tolist()[0], label='GPS-trajectory')
# ax.legend()
# ax.set_xlabel('x')
# ax.set_ylabel('y')
# # plt.show()

#-------------------------------------------------------------------------------------------------#

# read pose graph
pose_graph_file = "/home/shu/catkin_ws/src/VINS-Mono/pose_graph/TEST_1/pose_graph.txt"
timestamps_pg = []
PG_Tx = []
PG_Ty = []
PG_Tz = []
qx = []
qy = []
qz = []
qw = []
with open(pose_graph_file, 'r') as fp:
    temp = []
    for line in fp:
        # print(line)
        temp.append(line)

    for i in range(len(temp)):
        timestamps_pg.append(np.float(temp[i].split(" ")[2]))
        Tx = np.float(temp[i].split(" ")[6])
        Ty = np.float(temp[i].split(" ")[7])
        Tz = np.float(temp[i].split(" ")[8])

        PG_T = np.matrix([[Tx],
                          [Ty],
                          [Tz]])

        # Ry_minus90 = np.matrix([[0, 0, -1],
        #                         [0, 1, 0],
        #                         [1, 0, 0]])
        #
        # Rz_minus90 = np.matrix([[0, 1, 0],
        #                    [-1, 0, 0],
        #                    [0, 0, 1]])
        # Rx_minus30 = np.matrix([[1, 0, 0],
        #                        [0, np.cos(30*pi/180), -np.sin(30*pi/180)],
        #                        [0, np.sin(30*pi/180), np.cos(30*pi/180)]])

        # Rz_minus180 = np.matrix([[-1, 0, 0],
        #                    [0, -1, 0],
        #                    [0, 0, 1]])

        PG_T_ = PG_T
        #
        PG_Tx.append(PG_T_[0,0])
        PG_Ty.append(PG_T_[1,0])
        PG_Tz.append(PG_T_[2,0])


        qw.append(np.float(temp[i].split(" ")[13]))
        qx.append(np.float(temp[i].split(" ")[14]))
        qy.append(np.float(temp[i].split(" ")[15]))
        qz.append(np.float(temp[i].split(" ")[16]))

fig = plt.figure()
plt.plot(PG_Tx, PG_Ty, label='VINS-pose-graph')
plt.legend()
# ax.plot(GPS_position_in_mynteye_frame[0,:].tolist()[0], GPS_position_in_mynteye_frame[1,:].tolist()[0], GPS_position_in_mynteye_frame[2,:].tolist()[0], label='GPS-trajectory')
plt.plot(X, Y, '--', label='GPS')
plt.plot(PG_Tx[0], PG_Ty[0], 'rx', label='(VINS) start point')
plt.legend()
plt.plot(X[0], Y[0], 'bx', label='(GPS) start point')
plt.legend()
plt.grid(True)
plt.xlabel('x (easting) [m]')
plt.ylabel('y (northing) [m]')
plt.title('2D trajectory of GPS and pose graph (local frame of GPS)')
# plt.show()

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(PG_Tx, PG_Ty, PG_Tz, label='VINS-pose-graph')
ax.legend(loc='upper left')
# ax.plot(PG_Tx[0], PG_Ty[0], PG_Tz[0], 'rx', label='(VINS) start point')
# ax.legend()
# ax.plot(GPS_position_in_mynteye_frame[0,:].tolist()[0], GPS_position_in_mynteye_frame[1,:].tolist()[0], GPS_position_in_mynteye_frame[2,:].tolist()[0], label='GPS-trajectory')
ax.plot(X, Y, Z, '--', label='GPS')
ax.legend(loc='upper left')
# ax.plot(X[0], Y[0], Z[0], 'bx', label='(GPS) start point')
# ax.legend()
ax.set_xlabel('x (easting) [m]')
ax.set_ylabel('y (northing) [m]')
ax.set_zlabel('z (altitude) [m]')
ax.set_title('3D trajectory of GPS and pose graph (local frame of GPS)')
ax.autoscale()

# plt.figure()
# plt.subplot(211)
# plt.plot(timestamps_pvt, X)
# plt.subplot(212)
# plt.plot(timestamps_pg, PG_Tx)

plt.show()

t = [PG_Tx[0] - X[0], PG_Ty[0] - Y [0], PG_Tz[0] - Z[0]]
print(t)


# ----------------------------------------------------------------------------------------------------#
# interpolate GPS/IMU data to find corresponding measurement with the MYNT EYE camera timestamps
X_interp = np.interp(timestamps_pg, timestamps_pvt, X)
Y_interp = np.interp(timestamps_pg, timestamps_pvt, Y)
Z_interp = np.interp(timestamps_pg, timestamps_pvt, Z)


with open('./pose_graph_test.txt', 'w') as fp:
    for i in range(len(X_interp)):
        fp.write('%f %f %f %f %f %f %f %f \n' % (timestamps_pg[i], X_interp[i], Y_interp[i], Z_interp[i], qx[i], qy[i], qz[i], qw[i]))
