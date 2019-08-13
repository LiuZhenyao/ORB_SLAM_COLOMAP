import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numpy.linalg import inv
import quaternion
import pandas
import  gps_data_analysis

# for agricultural dataset
def agri_process():

    df = pandas.read_csv('/home/shu/Downloads/JD/2019_06_26_ExtractedTUKLData_Log123/2019_06_26_1428_57_extractedPoseData_StructImageGroup.csv')
    data = df[df['senID']==5]
    x = data['x'].tolist()
    y = data['y'].tolist()
    z = data['z'].tolist()

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
    for i in range(len(x)):
        coor = np.matrix([[x[i]],
                          [y[i]],
                          [z[i]]])
        # origin to sensor 5
        # coor_s = R.transpose() * coor - R.transpose() * t
        coor_s = coor

        X_.append(coor_s[0, 0])
        Y_.append(coor_s[1, 0])
        Z_.append(coor_s[2, 0])

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
    print(d)

    time = data['time'].tolist()
    softwareTimeStamp = data['softwareTimeStamp'].tolist()

    timestamps_gps = []
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


# for field data:
def field_process():
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

