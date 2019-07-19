import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import gps_data_analysis
from numpy.linalg import inv
import quaternion

csv_file_path = "/home/shu/Desktop/SF3000-recordings/2019_07_15_2047_17_perceptionLog_StructSensorSF3000.csv"
PVT_msg, INS_msg, IMU_msg = gps_data_analysis.read_gps_data(csv_file_path)
timestamps, easting, northing, altitude, velocityEasting, velocityNorthing, velocityUp = gps_data_analysis.sparse_PVT_msg(PVT_msg)

# Set first measurement of GPS as origin point
X = [(item - easting[0]) for item in easting]
Y = [(item - northing[0]) for item in northing]
Z = [(item - altitude[0]) for item in altitude]


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
pose_graph_file = "/home/shu/catkin_ws/src/VINS-Mono/pose_graph/TEST_3/pose_graph.txt"
with open(pose_graph_file, 'r') as fp:
    temp = []
    for line in fp:
        # print(line)
        temp.append(line)

    PG_Tx = []
    PG_Ty = []
    PG_Tz = []
    for i in range(len(temp)):
        Tx = np.float(temp[i].split(" ")[5])
        Ty = np.float(temp[i].split(" ")[6])
        Tz = np.float(temp[i].split(" ")[7])

        PG_T = np.matrix([[Tx],
                          [Ty],
                          [Tz]])

        Ry_minus90 = np.matrix([[0, 0, -1],
                                [0, 1, 0],
                                [1, 0, 0]])

        Rz_minus90 = np.matrix([[0, 1, 0],
                           [-1, 0, 0],
                           [0, 0, 1]])
        # Rx_minus30 = np.matrix([[1, 0, 0],
        #                        [0, np.cos(-30*pi/180), -np.sin(-30*pi/180)],
        #                        [0, np.sin(-30*pi/180), np.cos(-30*pi/180)]])

        PG_T_ = Rz_minus90 * Ry_minus90 * PG_T
        #
        PG_Tx.append(PG_T_[0,0])
        PG_Ty.append(PG_T_[1,0])
        PG_Tz.append(PG_T_[2,0])

#-------------------------------------------------------------------------------------------------#

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
ax.legend()
# ax.plot(PG_Tx[0], PG_Ty[0], PG_Tz[0], 'rx', label='(VINS) start point')
# ax.legend()
# ax.plot(GPS_position_in_mynteye_frame[0,:].tolist()[0], GPS_position_in_mynteye_frame[1,:].tolist()[0], GPS_position_in_mynteye_frame[2,:].tolist()[0], label='GPS-trajectory')
ax.plot(X, Y, Z, '--', label='GPS')
ax.legend()
# ax.plot(X[0], Y[0], Z[0], 'bx', label='(GPS) start point')
# ax.legend()
ax.set_xlabel('x (easting) [m]')
ax.set_ylabel('y (northing) [m]')
ax.set_zlabel('z (altitude) [m]')
ax.set_title('3D trajectory of GPS and pose graph (local frame of GPS)')
ax.autoscale()
plt.show()

t = [PG_Tx[0] - X[0], PG_Ty[0] - Y [0], PG_Tz[0] - Z[0]]
print(t)