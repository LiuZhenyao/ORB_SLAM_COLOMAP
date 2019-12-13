import gps_data_analysis
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy

def align(model, data):
    """Align two trajectories using the method of Horn (closed-form).

    Input:
    model -- first trajectory (3xn) SLAM
    data -- second trajectory (3xn) GT

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
gps_gt_path = "/home/shu/Downloads/Rosario_2019/dataset/03/gt.txt"
timestamps_gps = []
Xgps = []
Ygps = []
Zgps = []
Xc = []
Yc = []
Zc = []
with open(gps_gt_path, 'r') as fp:
    for line in fp:
        timestamps_gps.append(np.float(line.split(' ')[0]))

        Xgps.append(np.float(line.split(' ')[1]))
        Ygps.append(np.float(line.split(' ')[2]))
        Zgps.append(np.float(line.split(' ')[3]))


        gps_center = np.matrix([[np.float(line.split(' ')[1])],
                         [np.float(line.split(' ')[2])],
                         [np.float(line.split(' ')[3])],
                         [1.0]])

        # cam_center = T_imu_2_cam*(T_base_2_imu*(inv(T_base_2_gps)*gps_center))
        cam_center = gps_center

        Xc.append(cam_center[0, 0])
        Yc.append(cam_center[1, 0])
        Zc.append(cam_center[2, 0])

pose_graph_file = '/home/shu/dense_orbslam_ws/ORB_SLAM2_REMODE/backup_trajectory/KeyFrameTrajectory_03.txt'
timestamps_pg = []
tx = []
ty = []
tz = []
with open(pose_graph_file, 'r') as fp:
    for line in fp:
        timestamps_pg.append(np.float(line.split(' ')[0]))
        tx.append(np.float(line.split(' ')[1]))
        ty.append(np.float(line.split(' ')[2]))
        tz.append(np.float(line.split(' ')[3]))

X_interp = np.interp(timestamps_pg, timestamps_gps, Xc)
Y_interp = np.interp(timestamps_pg, timestamps_gps, Yc)
Z_interp = np.interp(timestamps_pg, timestamps_gps, Zc)

# Define two matrix
# GPS, Ground truth
first_xyz = np.matrix([[value for value in X_interp],
                       [value for value in Y_interp],
                       [value for value in Z_interp]])
# SLAM
second_xyz = np.matrix([[value for value in tx],
                       [value for value in ty],
                       [value for value in tz]])

rot,trans,trans_error,scale = align(second_xyz, first_xyz)

second_xyz_aligned = scale * rot * second_xyz + trans

print "compared_pose_pairs %d pairs" % (len(trans_error))

print "absolute_translational_error.rmse %f m" % numpy.sqrt(numpy.dot(trans_error, trans_error) / len(trans_error))
print "absolute_translational_error.mean %f m" % numpy.mean(trans_error)
print "absolute_translational_error.median %f m" % numpy.median(trans_error)
print "absolute_translational_error.std %f m" % numpy.std(trans_error)
print "absolute_translational_error.min %f m" % numpy.min(trans_error)
print "absolute_translational_error.max %f m" % numpy.max(trans_error)

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