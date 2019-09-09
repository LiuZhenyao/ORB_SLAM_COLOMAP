import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import gps_data_analysis
from numpy.linalg import inv
import quaternion
from scipy.interpolate import RegularGridInterpolator

## TODO: VINS + GPS, I was trying to combine GPS and mynteye IMU, see if VINS can calibrate extrinsic parameter online
## TODO: This idea is not success


csv_file_path = "/home/shu/Desktop/SF3000-recordings/2019_07_15_2047_17_perceptionLog_StructSensorSF3000.csv"
PVT_msg, INS_msg, IMU_msg = gps_data_analysis.read_gps_data(csv_file_path)
timestamps_imu, xAccel, yAccel, zAccel, xGyroRate, yGyroRate, zGyroRate = gps_data_analysis.sparse_IMU_msg(IMU_msg)
# timestamps_pvt, easting, northing, altitude, velocityEasting, velocityNorthing, velocityUp = gps_data_analysis.sparse_PVT_msg(PVT_msg)
#
# timestamps_pvt = np.asarray(timestamps_pvt)
# X = np.asarray([(item - easting[0]) for item in easting])
# Y = np.asarray([(item - northing[0]) for item in northing])
# Z = np.asarray([(item - altitude[0]) for item in altitude])

with open('/home/shu/kalibr_workspace/MYNTEYE_GPS_VINS_testdata/imu0.csv', 'a') as fp:
    for i in range(len(timestamps_imu)):
        tmp = ("%.9f" % timestamps_imu[i]).replace('.', '')
        print(tmp)
        fp.write('%s,%f,%f,%f,%f,%f,%f\n' % (tmp, xAccel[i], yAccel[i], zAccel[i], xGyroRate[i], yGyroRate[i], zGyroRate[i]))