import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numpy.linalg import inv
import quaternion
import pandas

df = pandas.read_csv('/home/shu/Downloads/JD/2019_06_26_ExtractedTUKLData_Log123/2019_06_26_1428_57_extractedPoseData_StructImageGroup.csv')
data = df[df['senID']==5]
x = data['x'].tolist()
y = data['y'].tolist()
z = data['z'].tolist()
X = [item - x[0] for item in x]
Y = [item - y[0] for item in y]
Z = [item - z[0] for item in z]

pose_graph_file = '/home/shu/OpensourceProject/MYNT-ORBSLAM2_ws/src/MYNT-EYE-ORB-SLAM2-Sample/pose_graph/sensor5_slot0/KeyFrameTrajectory.txt'
timestamps_pg = []
PG_Tx = []
PG_Ty = []
PG_Tz = []
with open(pose_graph_file, 'r') as fp:
    temp = []
    for line in fp:
        # print(line)
        temp.append(line)

    for i in range(len(temp)):
        ts = (temp[i].split(" ")[0]).split('.')
        timestamps_pg.append(np.float(ts[0] + ts[1]))
        PG_Tx.append(np.float(temp[i].split(" ")[1]))
        PG_Ty.append(np.float(temp[i].split(" ")[2]))
        PG_Tz.append(np.float(temp[i].split(" ")[3]))

# plt.figure()
# plt.subplot(211)
# plt.plot(timestamps_pg, PG_Tx)
# plt.subplot(212)
# plt.plot(data['softwareTimeStamp'], Y)
#
# plt.figure()
# plt.subplot(211)
# plt.plot(timestamps_pg, PG_Ty)
# plt.subplot(212)
# plt.plot(data['softwareTimeStamp'], X)
#
# plt.figure()
# plt.subplot(211)
# plt.plot(timestamps_pg, PG_Tz)
# plt.subplot(212)
# plt.plot(data['softwareTimeStamp'], Z)

# https://github.com/raulmur/ORB_SLAM2/issues/80

plt.figure()
plt.subplot(321)
plt.plot(timestamps_pg, PG_Tx, label='PG_Tx')
plt.legend()
plt.subplot(323)
plt.plot(timestamps_pg, PG_Tz, label='PG_Tz')
plt.legend()
plt.subplot(325)
plt.plot(timestamps_pg, PG_Ty, label='PG_Ty')
plt.legend()

plt.subplot(322)
plt.plot(data['softwareTimeStamp'], X, label='X')
plt.legend()
plt.subplot(324)
plt.plot(data['softwareTimeStamp'], Y, label='Y')
plt.legend()
plt.subplot(326)
plt.plot(data['softwareTimeStamp'], Z, label='Z')
plt.legend()


fig = plt.figure()
plt.plot(PG_Tx, PG_Tz, label='ORBSLAM-pose-graph')
plt.legend()
plt.plot(X, Y, '--', label='GPS')
plt.legend()
plt.plot(PG_Tx[0], PG_Ty[0], 'rx', label='(ORBSLAM) start point')
plt.legend()
plt.plot(X[0], Y[0], 'bx', label='(GPS) start point')
plt.legend()
plt.grid(True)
plt.xlabel('x (easting) [m]')
plt.ylabel('y (northing) [m]')

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(PG_Tx, PG_Ty, PG_Tz, label='ORBSLAM-pose-graph')
ax.legend(loc='upper left')
ax.plot(X, Y, Z, '--', label='GPS')
ax.legend(loc='upper left')
ax.set_xlabel('x (easting) [m]')
ax.set_ylabel('y (northing) [m]')
ax.set_zlabel('z (altitude) [m]')
ax.set_title('3D trajectory of GPS and pose graph (local frame of GPS)')
ax.autoscale()

plt.show()
