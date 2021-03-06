import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numpy.linalg import inv
import quaternion
import pandas
import  gps_data_analysis
from math import sqrt
from numpy import *


def GeoRegistration_agri():
    # Geo-Registration: agricultural dataset from John Deere
    csv_path = '/home/shu/Downloads/JD/2019_09_06_0938_48_extractedPoseData_StructImageGroup.csv'
    df = pandas.read_csv(csv_path)
    data = df[df['senID']==5]  # sensor 5
    x = data['x'].tolist()
    y = data['y'].tolist()
    z = data['z'].tolist()
    # roll = data['roll'].tolist()
    # pitch = data['pitch'].tolist()
    # yaw = data['yaw'].tolist()

    # Sensor 5 to origin:
    R = np.matrix([[-0.6459666, -0.7385558, -0.1930348],
                   [-0.2928724, 0.4732986, -0.8307913],
                   [0.7049489, -0.4801289, -0.5220377]])

    t = np.matrix([[2.6655631],
                   [2.2556686],
                   [0.0602708]])

    # Sensor 6 to origin
    # R = np.matrix([[0.6333929, -0.7428564, 0.2167435],
    #                [-0.2918709, -0.4887433, -0.8221565],
    #                [0.7166762, 0.4574870, -0.5263847]])
    #
    # t = np.matrix([[-2.5567965],
    #                [2.2590834],
    #                [0.0541633]])


    X_ = []
    Y_ = []
    Z_ = []

    for i in range(len(x)):
        coor = np.matrix([[x[i]],
                          [y[i]],
                          [z[i]]])
        # origin to sensor 5
        coor_s = R.transpose() * coor - R.transpose() * t
        # coor_s = coor

        X_.append(coor_s[0, 0])
        Y_.append(coor_s[1, 0])
        Z_.append(coor_s[2, 0])

    # set first point as origin
    X = [item - X_[0] for item in X_]
    Y = [item - Y_[0] for item in Y_]
    Z = [item - Z_[0] for item in Z_]

    plt.figure()
    plt.plot(X, Y, label='GPS')
    plt.title('Trajectory of GPS (local frame)')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.legend()
    plt.grid(True)
    plt.show()

    d = np.sqrt((X[0]-X[-1])*(X[0]-X[-1]) + (Y[0]-Y[-1])*(Y[0]-Y[-1]))
    print("2D distance of gps trajectory: ", d)

    time = data['time'].tolist()
    softwareTimeStamp = data['softwareTimeStamp'].tolist()

    timestamps_gps = []
    ## TODO: use new csv file, ignore software and hardware time
    for i in range(len(time)):
        timestamps_gps.append(np.double(str(time[i]) + '.' + str(softwareTimeStamp[i]).replace('.', '')))

    pose_graph_file = '/home/shu/OpensourceProject/MYNT-ORBSLAM2_ws/src/MYNT-EYE-ORB-SLAM2-Sample/pose_graph/sensor5_slot0/KeyFrameTrajectory.txt'
    timestamps_pg = []
    qx = []
    qy = []
    qz = []
    qw = []
    PG_Tx = []
    PG_Ty = []
    PG_Tz = []
    with open(pose_graph_file, 'r') as fp:
        temp = []
        for line in fp:
            temp.append(line)

        for i in range(len(temp)):
            ts = (temp[i].split(" ")[0]).split('.')
            ts_1 = ts[0] + ts[1]
            ts_2 = ts_1[0:10] + '.' + ts_1[10:]
            timestamps_pg.append(np.float(ts_2))
            PG_Tx.append(np.float(temp[i].split(" ")[1]))
            PG_Ty.append(np.float(temp[i].split(" ")[2]))
            PG_Tz.append(np.float(temp[i].split(" ")[3]))
            qx.append(np.float(temp[i].split(" ")[4]))
            qy.append(np.float(temp[i].split(" ")[5]))
            qz.append(np.float(temp[i].split(" ")[6]))
            qw.append(np.float(temp[i].split(" ")[7]))

    X_interp = np.interp(timestamps_pg, timestamps_gps, X)
    Y_interp = np.interp(timestamps_pg, timestamps_gps, Y)
    Z_interp = np.interp(timestamps_pg, timestamps_gps, Z)

    # "ref_images_agri.txt":
    #   (used in COLMAP for geo-registration)
    #   IMG_name X Y Z
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



def GeoRegistration_field():
    # Geo-Registration: field data captured by mynteye and starfire
    kf_file = '/home/shu/OpensourceProject/MYNT-ORBSLAM2_ws/src/MYNT-EYE-ORB-SLAM2-Sample/pose_graph/KeyFrameTrajectory.txt'
    timestamps_pg = []
    with open(kf_file, 'r') as fp:
        for line in fp:
            timestamps_pg.append(line.split(' ')[0])

    csv_file_path = "/home/shu/Desktop/SF3000-recordings/2019_07_15_2047_17_perceptionLog_StructSensorSF3000.csv"
    PVT_msg, INS_msg, IMU_msg = gps_data_analysis.read_gps_data(csv_file_path)
    timestamps_pvt, easting, northing, altitude, velocityEasting, velocityNorthing, velocityUp = gps_data_analysis.sparse_PVT_msg(
        PVT_msg)

    # Set first measurement of GPS as origin point
    timestamps_pvt = np.asarray(timestamps_pvt)
    X = np.asarray([(item - easting[0]) for item in easting])
    Y = np.asarray([(item - northing[0]) for item in northing])
    Z = np.asarray([(item - altitude[0]) for item in altitude])

    plt.plot(X, Y)

    X_interp = np.interp(timestamps_pg, timestamps_pvt, X)
    Y_interp = np.interp(timestamps_pg, timestamps_pvt, Y)
    Z_interp = np.interp(timestamps_pg, timestamps_pvt, Z)

    with open('./ref_images_field.txt', 'w') as fp:
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


if __name__ == "__main__":

    kf_file = '/home/shu/dense_orbslam_ws/ORB_SLAM2/KeyFrameTrajectory_1.txt'
    timestamps_pg = []
    tx = []
    ty = []
    tz = []
    qx = []
    qy = []
    qz = []
    qw = []
    with open(kf_file, 'r') as fp:
        for line in fp:
            # print(line)
            timestamps_pg.append(line.split(' ')[0])

            t_ = np.matrix([[np.float(line.split(' ')[1])],
                           [np.float(line.split(' ')[2])],
                           [np.float(line.split(' ')[3])]])

            t = t_

            tx.append(t[0, 0])
            ty.append(t[1, 0])
            tz.append(t[2, 0])

            qx.append(np.float(line.split(' ')[4]))
            qy.append(np.float(line.split(' ')[5]))
            qz.append(np.float(line.split(' ')[6]))
            qw.append(np.float(line.split(' ')[7].split('\n')[0]))

    csv_file_path = "/home/shu/Desktop/SF3000-recordings/2019_07_12_0955_54_perceptionLog_StructSensorSF3000.csv"
    # csv_file_path = "/home/shu/Desktop/SF3000-recordings/2019_07_15_1100_45_perceptionLog_StructSensorSF3000.csv"
    # csv_file_path = "/home/shu/Desktop/SF3000-recordings/2019_07_15_1102_46_perceptionLog_StructSensorSF3000.csv"
    PVT_msg, INS_msg, IMU_msg = gps_data_analysis.read_gps_data(csv_file_path)

    timestamps_pvt, easting, northing, altitude, velocityEasting, velocityNorthing, velocityUp = gps_data_analysis.sparse_PVT_msg(
        PVT_msg)

    # Set first measurement of GPS as origin point
    timestamps_pvt = np.asarray(timestamps_pvt)
    X = np.asarray([(item) for item in easting])
    Y = np.asarray([(item) for item in northing])
    Z = np.asarray([(item) for item in altitude])

    X_interp_ = np.interp(timestamps_pg, timestamps_pvt, X)
    Y_interp_ = np.interp(timestamps_pg, timestamps_pvt, Y)
    Z_interp_ = np.interp(timestamps_pg, timestamps_pvt, Z)

    X_interp = [(item - X_interp_[0]) for item in X_interp_]
    Y_interp = [(item - Y_interp_[0]) for item in Y_interp_]
    Z_interp = [(item - Z_interp_[0]) for item in Z_interp_]

    # find 3D transformation https://nghiaho.com/?page_id=671
    A = np.zeros((len(tx),3))
    B = np.zeros((len(tx),3))

    for i in range(len(tx)):
        A[i, 0] = tx[i]
        A[i, 1] = ty[i]
        A[i, 2] = tz[i]

        B[i, 0] = X_interp[i]
        B[i, 1] = Y_interp[i]
        B[i, 2] = Z_interp[i]

    R, t = Rigid_Transform_3D(A, B)

    timestamps_pg = []
    tx = []
    ty = []
    tz = []
    qx = []
    qy = []
    qz = []
    qw = []
    with open(kf_file, 'r') as fp:
        for line in fp:
            print(line)
            timestamps_pg.append(line.split(' ')[0])

            t_ = np.matrix([[np.float(line.split(' ')[1])],
                           [np.float(line.split(' ')[2])],
                           [np.float(line.split(' ')[3])]])

            t = R*t_

            tx.append(t[0, 0])
            ty.append(t[1, 0])
            tz.append(t[2, 0])

            qx.append(np.float(line.split(' ')[4]))
            qy.append(np.float(line.split(' ')[5]))
            qz.append(np.float(line.split(' ')[6]))
            qw.append(np.float(line.split(' ')[7].split('\n')[0]))

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(tx, ty, tz, label='ORB-pose-graph')
    ax.plot(X_interp,Y_interp,Z_interp, label='GPS')
    plt.legend(loc='best')
    plt.grid(True)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.set_title('3D trajectory of GPS and pose graph (local frame of GPS)')
    ax.autoscale()

    plt.figure()
    plt.plot(tx, ty, label="ORB-SLAM pose graph")
    plt.plot(X_interp, Y_interp, 'g--', label="GPS")
    plt.plot(X_interp[0], Y_interp[0], 'bx', label='(GPS) start point')
    plt.plot(tx[0], tx[0], 'rx', label='(SLAM) start point')
    plt.legend()
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.grid(True)
    plt.title('2D trajectory of GPS and pose graph (local frame of GPS)')
    plt.show()


    # with open('./pose_graph_test_orb.txt', 'w') as fp:
    #     for i in range(len(X_interp)):
    #         fp.write('%s %f %f %f %f %f %f %f \n' % (timestamps_pg[i], X_interp[i], Y_interp[i], Z_interp[i], qx[i], qy[i], qz[i], qw[i]))