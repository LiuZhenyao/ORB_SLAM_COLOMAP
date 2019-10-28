import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import gps_data_analysis
from numpy.linalg import inv
import quaternion
from scipy.interpolate import RegularGridInterpolator
from numpy import *

def Rigid_Transform_3D(A, B):

    # Input: expects Nx3 matrix of points
    # Returns R,t
    # R = 3x3 rotation matrix
    # t = 3x1 column vector

    assert len(A) == len(B)

    N = A.shape[0]  # total points

    centroid_A = mean(A, axis=0)
    centroid_B = mean(B, axis=0)

    # centre the points
    AA = np.asmatrix(A - tile(centroid_A, (N, 1)))
    BB = np.asmatrix(B - tile(centroid_B, (N, 1)))

    # dot is matrix multiplication for array
    H = np.matmul(AA.T, BB)

    U, S, Vt = linalg.svd(H)

    R = Vt.T * U.T

    # special reflection case
    if linalg.det(R) < 0:
        print "Reflection detected"
        Vt[2, :] *= -1
        R = Vt.T * U.T

    t = -R * asmatrix(centroid_A).T + asmatrix(centroid_B).T

    print t

    return R, t


# csv_file_path = "/home/shu/Desktop/SF3000-recordings/2019_07_12_0955_54_perceptionLog_StructSensorSF3000.csv"
# csv_file_path = "/home/shu/Desktop/SF3000-recordings/2019_07_15_1100_45_perceptionLog_StructSensorSF3000.csv"
csv_file_path = "/home/shu/Desktop/SF3000-recordings/2019_07_15_1102_46_perceptionLog_StructSensorSF3000.csv"
PVT_msg, INS_msg, IMU_msg = gps_data_analysis.read_gps_data(csv_file_path)
timestamps_pvt, easting, northing, altitude, velocityEasting, velocityNorthing, velocityUp = gps_data_analysis.sparse_PVT_msg(PVT_msg)

# # Set first measurement of GPS as origin point
# timestamps_pvt = np.asarray(timestamps_pvt)[4500:-1]
# X = np.asarray([(item - easting[4500]) for item in easting])[4500:-1]
# Y = np.asarray([(item - northing[4500]) for item in northing])[4500:-1]
# Z = np.asarray([(item - altitude[4500]) for item in altitude])[4500:-1]

timestamps_pvt = np.asarray(timestamps_pvt)
X = np.asarray([(item) for item in easting])
Y = np.asarray([(item) for item in northing])
Z = np.asarray([(item) for item in altitude])

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

        PG_T_ = PG_T
        #
        PG_Tx.append(PG_T_[0,0])
        PG_Ty.append(PG_T_[1,0])
        PG_Tz.append(PG_T_[2,0])


        qw.append(np.float(temp[i].split(" ")[13]))
        qx.append(np.float(temp[i].split(" ")[14]))
        qy.append(np.float(temp[i].split(" ")[15]))
        qz.append(np.float(temp[i].split(" ")[16]))


X_interp_ = np.interp(timestamps_pg, timestamps_pvt, X)
Y_interp_ = np.interp(timestamps_pg, timestamps_pvt, Y)
Z_interp_ = np.interp(timestamps_pg, timestamps_pvt, Z)

X_interp = [(item - X_interp_[0]) for item in X_interp_]
Y_interp = [(item - Y_interp_[0]) for item in Y_interp_]
Z_interp = [(item - Z_interp_[0]) for item in Z_interp_]


# find 3D transformation https://nghiaho.com/?page_id=671
A = np.zeros((len(PG_Tx),3))
B = np.zeros((len(PG_Tx),3))

for i in range(len(PG_Tx)):
    A[i, 0] = PG_Tx[i]
    A[i, 1] = PG_Ty[i]
    A[i, 2] = PG_Tz[i]

    B[i, 0] = X_interp[i]
    B[i, 1] = Y_interp[i]
    B[i, 2] = Z_interp[i]

R, t = Rigid_Transform_3D(A, B)

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

        PG_T_ = R*PG_T
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
plt.plot(X_interp, Y_interp, 'g--', label='GPS')
plt.plot(PG_Tx[0], PG_Ty[0], 'rx', label='(VINS) start point')
plt.plot(X_interp[0], Y_interp[0], 'bx', label='(GPS) start point')
plt.legend()
plt.grid(True)
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('2D trajectory of GPS and pose graph (local frame of GPS)')
# plt.show()

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(PG_Tx, PG_Ty, PG_Tz, label='VINS-pose-graph')
ax.plot(X_interp, Y_interp, Z_interp, 'g--', label='GPS')
ax.legend(loc='best')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_zlabel('z [m]')
ax.set_title('3D trajectory of GPS and pose graph')
ax.autoscale()

plt.show()

t = [PG_Tx[0] - X_interp[0], PG_Ty[0] - Y_interp[0], PG_Tz[0] - Z_interp[0]]
print(t)


# ----------------------------------------------------------------------------------------------------#
# interpolate GPS/IMU data to find corresponding measurement with the MYNT EYE camera timestamps
X_interp = np.interp(timestamps_pg, timestamps_pvt, X)
Y_interp = np.interp(timestamps_pg, timestamps_pvt, Y)
Z_interp = np.interp(timestamps_pg, timestamps_pvt, Z)

#
# with open('./pose_graph_test.txt', 'w') as fp:
#     for i in range(len(X_interp)):
#         fp.write('%f %f %f %f %f %f %f %f \n' % (timestamps_pg[i], X_interp[i], Y_interp[i], Z_interp[i], qx[i], qy[i], qz[i], qw[i]))
