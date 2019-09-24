import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numpy.linalg import inv
import pandas
import gps_data_analysis
from math import sqrt
from numpy import *
import os
import cv2

# pkg quaternion: https://pypi.org/project/numpy-quaternion/
# https://quaternion.readthedocs.io/en/latest/README.html#basic-usage
import quaternion


def EulerAngles_to_Rotation(roll, pitch, yaw):
    """
    :param roll:
    :param pitch:
    :param yaw:
    :return: R: rotation matrix
    """

    yawMatrix = np.matrix([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    pitchMatrix = np.matrix([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    rollMatrix = np.matrix([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    return yawMatrix * pitchMatrix * rollMatrix


def create_test_data_for_REMODE():

    """
    According to the test_data of REMODE: https://github.com/uzh-rpg/rpg_open_remode/wiki/Test-REMODE:
    create similar test data from agricultural image sequence, which is directly geo-referenced by GPS
    """

    csv_path = '/home/shu/Downloads/JD/2019_09_06_0938_48_extractedPoseData_StructImageGroup.csv'
    df = pandas.read_csv(csv_path)
    data = df[df['senID']==5]  # sensor 5
    x = data['x'].tolist()
    y = data['y'].tolist()
    z = data['z'].tolist()
    roll = data['roll'].tolist()
    pitch = data['pitch'].tolist()
    yaw = data['yaw'].tolist()

    timestamps = data['time'].tolist()
    timestamps.sort()

    # Sensor 5 to origin:
    R = np.matrix([[-0.6459666, -0.7385558, -0.1930348],
                   [-0.2928724, 0.4732986, -0.8307913],
                   [0.7049489, -0.4801289, -0.5220377]])

    t = np.matrix([[2.6655631],
                   [2.2556686],
                   [0.0602708]])

    X_ = []
    Y_ = []
    Z_ = []
    qx = []
    qy = []
    qz = []
    qw = []
    for i in range(len(x)):
        # define instant GPS (it is also origin) coordinates and orientations
        coor_gps = np.matrix([[x[i]],
                          [y[i]],
                          [z[i]]])
        R_gps = EulerAngles_to_Rotation(roll[i], pitch[i], yaw[i])

        # transformation from origin to sensor (sensor 5 ot 6)
        # coor_s = R.transpose() * coor_gps - R.transpose() * t
        coor_s = coor_gps
        R_s =  R.transpose() * R_gps

        #
        X_.append(coor_s[0, 0])
        Y_.append(coor_s[1, 0])
        Z_.append(coor_s[2, 0])

        q = quaternion.from_rotation_matrix(R_s)
        qw.append(q.w)
        qx.append(q.x)
        qy.append(q.y)
        qz.append(q.z)

    X = [item - X_[0] for item in X_]
    Y = [item - Y_[0] for item in Y_]
    Z = [item - Z_[0] for item in Z_]

    # plt.figure()
    # plt.plot(x, y, label='GPS')
    # plt.title('Trajectory of GPS (local frame)')
    # plt.xlabel('x [m]')
    # plt.ylabel('y [m]')
    # plt.legend()
    # plt.grid(True)
    #
    # fig = plt.figure()
    # ax = fig.gca(projection='3d')
    # ax.plot(X, Y, Z, label='GPS')
    # plt.legend(loc='upper left')
    # ax.set_xlabel('x ')
    # ax.set_ylabel('y ')
    # ax.set_zlabel('z ')
    # ax.set_title('3D trajectory of GPS and pose graph (local frame of GPS)')
    # ax.autoscale()
    # plt.show()

    img_list = os.listdir("/home/shu/SVO_ws/src/rpg_open_remode/test_data_agri/images")
    img_list.sort()

    with open('./remode_data_test.txt', 'w') as fp:
        for i in range(0, 335):
            fp.write('%s %f %f %f %f %f %f %f \n' % (img_list[i], X[i], Y[i], Z[i], qx[i], qy[i], qz[i], qw[i]))


def depthmap_to_32FC1():

    path_left = '/home/shu/SVO_ws/src/rpg_open_remode/test_data_agri/orb_slot_0'
    path_right = '/home/shu/SVO_ws/src/rpg_open_remode/test_data_agri/orb_slot_1'
    img_list_L = os.listdir(path_left)
    img_list_L.sort()
    img_list_R = os.listdir(path_right)
    img_list_R.sort()

    for i in range(len(img_list_L)):
        assert img_list_L[i] == img_list_R[i]

        imgL = cv2.imread(os.path.join(path_left, img_list_L[i]), 0)
        imgR = cv2.imread(os.path.join(path_right, img_list_R[i]), 0)
        win_size = 5
        min_disp = -1
        max_disp = 63  # min_disp * 9
        num_disp = max_disp - min_disp  # Needs to be divisible by 1
        stereo = cv2.StereoSGBM_create(minDisparity=min_disp,
                                       numDisparities=num_disp,
                                       blockSize=5,
                                       uniquenessRatio=5,
                                       speckleWindowSize=5,
                                       speckleRange=5,
                                       disp12MaxDiff=1,
                                       P1=8 * 3 * win_size ** 2,  # 8*3*win_size**2,
                                       P2=32 * 3 * win_size ** 2)  # 32*3*win_size**2)
        disparity = stereo.compute(imgL, imgR)
        disparity[disparity == -32] = 1
        disparity[disparity == 0] = 1
        # plt.imshow(disparity, 'gray')
        # plt.show()

        np.savetxt(os.path.join('/home/shu/SVO_ws/src/rpg_open_remode/test_data_agri/depthmaps', img_list_L[i].replace('png','depth')), disparity)



    # baseline = 0.3 # [m]
    # focal = 912 # pixel
    # depth = baseline * focal / disparity
    #
    #
    #
    # plt.imshow(depth, 'gray')
    # plt.show()
    #
    # np.savetxt('./text.depth', depth)



    