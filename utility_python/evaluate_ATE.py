import gps_data_analysis
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy

def align(model, data):
    """Align two trajectories using the method of Horn (closed-form).

    Input:
    model -- first trajectory (3xn) GT
    data -- second trajectory (3xn) SLAM

    Output:
    rot -- rotation matrix (3x3)
    trans -- translation vector (3x1)
    trans_error -- translational error per point (1xn)

    """
    numpy.set_printoptions(precision=3, suppress=True)
    model_zerocentered = model - model.mean(1)
    data_zerocentered = data - data.mean(1)

    W = numpy.zeros((3, 3))
    for column in range(model.shape[1]):
        W += numpy.outer(model_zerocentered[:, column], data_zerocentered[:, column])
    U, d, Vh = numpy.linalg.linalg.svd(W.transpose())
    S = numpy.matrix(numpy.identity(3))
    if (numpy.linalg.det(U) * numpy.linalg.det(Vh) < 0):
        S[2, 2] = -1
    rot = U * S * Vh

    rotmodel = rot * model_zerocentered
    dots = 0.0
    norms = 0.0

    for column in range(data_zerocentered.shape[1]):
        dots += numpy.dot(data_zerocentered[:, column].transpose(), rotmodel[:, column])
        normi = numpy.linalg.norm(model_zerocentered[:, column])
        norms += normi * normi

    s = float(dots / norms)

    print "scale: %f " % s

    trans = data.mean(1) - s * rot * model.mean(1)

    model_aligned = s * rot * model + trans
    alignment_error = model_aligned - data

    trans_error = numpy.sqrt(numpy.sum(numpy.multiply(alignment_error, alignment_error), 0)).A[0]

    return rot, trans, trans_error, s

# GPS data
# csv_file_path = "/home/shu/Desktop/SF3000-recordings/2019_07_12_0955_54_perceptionLog_StructSensorSF3000.csv"
# csv_file_path = "/home/shu/Desktop/SF3000-recordings/2019_07_15_1100_45_perceptionLog_StructSensorSF3000.csv"
csv_file_path = "/home/shu/Desktop/SF3000-recordings/2019_07_15_1102_46_perceptionLog_StructSensorSF3000.csv"
PVT_msg, INS_msg, IMU_msg = gps_data_analysis.read_gps_data(csv_file_path)
timestamps_pvt, easting, northing, altitude, velocityEasting, velocityNorthing, velocityUp = gps_data_analysis.sparse_PVT_msg(PVT_msg)
timestamps_pvt = np.asarray(timestamps_pvt)
X = np.asarray([item for item in easting])
Y = np.asarray([item for item in northing])
Z = np.asarray([item for item in altitude])

# Pose graph of ORB-SLAM2
kf_file = '/home/shu/dense_orbslam_ws/ORB_SLAM2/KeyFrameTrajectory_3.txt'
timestamps_pg = []
tx = []
ty = []
tz = []
with open(kf_file, 'r') as fp:
    for line in fp:
        timestamps_pg.append(line.split(' ')[0])
        tx.append(np.float(line.split(' ')[1]))
        ty.append(np.float(line.split(' ')[2]))
        tz.append(np.float(line.split(' ')[3]))

# Interpolate corresponding GPS coordinates according to timestamps
X_interp_ = np.interp(timestamps_pg, timestamps_pvt, X)
Y_interp_ = np.interp(timestamps_pg, timestamps_pvt, Y)
Z_interp_ = np.interp(timestamps_pg, timestamps_pvt, Z)
# Re-center
X_interp = [(item - X_interp_[0]) for item in X_interp_]
Y_interp = [(item - Y_interp_[0]) for item in Y_interp_]
Z_interp = [(item - Z_interp_[0]) for item in Z_interp_]

# Define two matrix
# GPS
first_xyz = np.matrix([[value for value in X_interp],
                       [value for value in Y_interp],
                       [value for value in Z_interp]])
# SLAM
second_xyz = np.matrix([[value for value in tx],
                       [value for value in ty],
                       [value for value in tz]])

rot,trans,trans_error,scale = align(second_xyz,first_xyz)

second_xyz_aligned = scale * rot * second_xyz + trans


# Plot
x = []
y = []
z = []
for i in range(second_xyz_aligned.shape[1]):
    x.append(second_xyz_aligned[0, i])
    y.append(second_xyz_aligned[1, i])
    z.append(second_xyz_aligned[2, i])


plt.figure()
plt.plot(x, y, label="ORB-SLAM pose graph")
plt.plot(X_interp, Y_interp, 'g--', label="GPS")
plt.legend()
plt.xlabel('x (easting) [m]')
plt.ylabel('y (northing) [m]')
plt.grid(True)
plt.title('2D trajectory of GPS and pose graph')

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(x, y, z, label='ORB-SLAM pose-graph')
ax.plot(X_interp, Y_interp, Z_interp, 'g--', label='GPS')
plt.legend(loc='best')
plt.grid(True)
ax.set_xlabel('x (easting) [m]')
ax.set_ylabel('y (northing) [m]')
ax.set_zlabel('z (altitude) [m]')
ax.set_title('3D trajectory of GPS and pose graph')
ax.autoscale()

plt.show()