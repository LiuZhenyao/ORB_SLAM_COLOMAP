import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import quaternion
from numpy.linalg import inv

# geo-registration of Rosario dataset, sequence 04

# transformation between GPS and camera
# https://www.cifasis-conicet.gov.ar/robot/doku.php
# GPS -> base_link -> imu -> left_camera
# TODO: is that necessary?
# TODO: cannot get the right camera center coordinates
# TODO: now, I can only consider translations, or even not necessary because scale will not change
# TODO: use the GPS directly
# quaternion please use https://quaternion.readthedocs.io/en/latest/README.html#basic-usage

# imu -> camera
# T_imu_2_cam = np.matrix([[0.010486169744957197, 0.9972728488827708, 0.07305412462908822, -0.04365571656478627],
#                          [-0.9940873423040585, 0.002500626397312078, 0.10855460717295605, -0.033255842186752066],
#                          [0.10807588128224391, -0.07376050263429253, 0.9914025378907414, -0.02682521301910111],
#                          [0.0, 0.0, 0.0, 1.0]])
#
# # base -> imu
# q_base_2_imu = np.quaternion(7.16012579e-04, -7.84171987e-01, -7.88065033e-04, -6.20542634e-01) # quaternion(w, x, y, z)
# R_base_2_imu = quaternion.as_rotation_matrix(q_base_2_imu)
# t_base_2_imu = np.array([1.96, -0.017, 1.02, 1.0])
# T_base_2_imu = np.zeros((4, 4), np.float)
# T_base_2_imu[:3, :3] = R_base_2_imu
# T_base_2_imu[:, 3] = t_base_2_imu
# T_base_2_imu = np.asmatrix(T_base_2_imu)
#
# # base -> gps
# q_base_2_gps = np.quaternion(1, 0, 0, 0) # quaternion(w, x, y, z)
# R_base_2_gps = quaternion.as_rotation_matrix(q_base_2_gps)
# t_base_2_gps = np.array([1.80070337, -0.02982362, 1.59345719, 1.0])
# T_base_2_gps = np.zeros((4, 4), np.float)
# T_base_2_gps[:3, :3] = R_base_2_gps
# T_base_2_gps[:, 3] = t_base_2_gps
# T_base_2_gps = np.asmatrix(T_base_2_gps)



# read gps data: timestamp, x, y, z
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

# read pose graph of ORB-SLAM
pose_graph_file = '/home/shu/dense_orbslam_ws/ORB_SLAM2_REMODE/backup_trajectory/KeyFrameTrajectory_03.txt'
timestamps_pg = []
with open(pose_graph_file, 'r') as fp:
    for line in fp:
        timestamps_pg.append(np.float(line.split(' ')[0]))

X_interp = np.interp(timestamps_pg, timestamps_gps, Xc)
Y_interp = np.interp(timestamps_pg, timestamps_gps, Yc)
Z_interp = np.interp(timestamps_pg, timestamps_gps, Zc)

with open('./ref_images_agri.txt', 'w') as fp:
    for i in range(len(X_interp)):
        img_name = ""
        if i < 10:
            img_name = "00000" + str(i)
        elif i > 9 and i < 100:
            img_name = "0000" + str(i)
        elif i > 99 and i < 1000:
            img_name = "000" + str(i)
        elif i > 999 and i < 10000:
            img_name = "00" + str(i)
        elif i > 9999 and i < 100000:
            img_name = "0" + str(i)
        fp.write('%s %f %f %f \n' % ('IMG' + img_name + '.png', X_interp[i], Y_interp[i], Z_interp[i]))


fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(X_interp, Y_interp, Z_interp, label='registered-camera')
ax.plot(Xgps, Ygps, Zgps, label='GPS')
plt.legend(loc='best')
plt.grid(True)
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_zlabel('z [m]')
ax.set_title('3D trajectory of GPS and pose graph (local frame of GPS)')
ax.autoscale()

plt.figure()
plt.plot(X_interp, Y_interp, label="ORB-SLAM pose graph")
plt.plot(Xgps, Ygps, 'g--', label="GPS")
plt.plot(Xgps[0], Ygps[0], 'bx', label='(GPS) start point')
plt.legend()
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.grid(True)
plt.title('2D trajectory of GPS and pose graph (local frame of GPS)')

plt.show()