import pandas
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os

# read and sparse GPS data file "2019_09_06_0938_48_extractedPoseData_StructImageGroup.csv"

df = pandas.read_csv('/home/shu/Downloads/JD/2019_06_26_ExtractedTUKLData_Log123/2019_06_26_1428_57_extractedPoseData_StructImageGroup.csv')

data = df[df['senID']==5]
#
#
#
# x = data['x'].tolist()
# y = data['y'].tolist()
#
# X = [item - x[0] for item in x]
# Y = [item - y[0] for item in y]
#
# d = np.sqrt((X[0]-X[-1])*(X[0]-X[-1]) + (Y[0]-Y[-1])*(Y[0]-Y[-1]))
#
#
# plt.figure()
# plt.plot(X, Y, label='sensor 5, slot 0')
# plt.title('Trajectory of sensor 5, slot 0 (local frame)')
# plt.xlabel('x [m]')
# plt.ylabel('y [m]')
# plt.legend()
# plt.grid(True)
#
# plt.show()


def agri_to_vins():

    df = pandas.read_csv(
        '/home/shu/Downloads/JD/2019_06_26_ExtractedTUKLData_Log123/2019_06_26_1428_57_extractedPoseData_StructImageGroup.csv')

    data = df[df['senID'] == 5]
    x = data['x'].tolist()
    y = data['y'].tolist()
    z = data['z'].tolist()
    x_ = [item - x[0] for item in x]
    y_ = [item - y[0] for item in y]
    z_ = [item - z[0] for item in z]
    roll = data['roll'].tolist()
    pitch = data['pitch'].tolist()
    yaw = data['yaw'].tolist()


    time = data['time'].tolist()
    softwareTimeStamp = data['softwareTimeStamp'].tolist()

    timestamps_gps = []
    for i in range(len(time)):
        timestamps_gps.append(np.double(str(time[i]) + '.' + str(softwareTimeStamp[i]).replace('.', '')))
    timestamps_gps.sort()


    img_list = os.listdir("/home/shu/Downloads/JD/2019_06_26_ExtractedTUKLData_Log123/Sensor_5/orb_slot_0")
    timestamps_img = []
    for name in img_list:
        tmp = name.split(".")[0]
        tmp_ = tmp[:-4] + "." + tmp[-4:]
        timestamps_img.append(np.float64(tmp_))
    timestamps_img.sort()


    X_interp = np.interp(timestamps_img, timestamps_gps, x_)
    Y_interp = np.interp(timestamps_img, timestamps_gps, y_)
    Z_interp = np.interp(timestamps_img, timestamps_gps, z_)

    roll_interp = np.interp(timestamps_img, timestamps_gps, roll)
    pitch_interp = np.interp(timestamps_img, timestamps_gps, pitch)
    yaw_interp = np.interp(timestamps_img, timestamps_gps, yaw)

    #
    # plt.figure()
    # plt.plot(timestamps_img, yaw_interp)
    # plt.figure()
    # plt.plot(timestamps_gps, yaw)

    a_x = []
    a_y = []
    a_z = []
    roll_rate = []
    pitch_rate = []
    yaw_rate = []
    for i in range(len(timestamps_img)-1):
        t = (timestamps_img[i+1] - timestamps_img[i])

        d_x = X_interp[i+1] - X_interp[i]
        v_x = d_x/t

        a_x.append(v_x/t)

        d_y = Y_interp[i+1] - Y_interp[i]
        v_y = d_y/t
        a_y.append(v_y/t)

        d_z = Z_interp[i+1] - Z_interp[i]
        v_z = d_z/t
        a_z.append(v_z/t)

        print(np.sqrt(v_x**2 + v_y**2 + v_z**2))

        roll_ = roll_interp[i+1] - roll_interp[i]
        roll_rate.append(roll_/t)

        yaw_ = yaw_interp[i + 1] - yaw_interp[i]
        yaw_rate.append(yaw_ / t)

        pitch_ = pitch_interp[i + 1] - pitch_interp[i]
        pitch_rate.append(pitch_ / t)
