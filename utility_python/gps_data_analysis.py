# read and sparse Starfire 3000/6000 GPS csv file
# msg rate 10 Hz

import numpy as np
import matplotlib.pyplot as plt

def read_gps_data(csv_file_path):
    PVT_msg = []
    INS_msg = []
    IMU_msg = []
    with open(csv_file_path, 'r') as fp:
        line_count = 0
        for line in fp:
            if line_count >= 3:
                # print (line)  # string
                sparse_data = line.split(",")
                if int(sparse_data[5]) == 1: # PVT
                    PVT_msg.append(line)
                elif int(sparse_data[5]) == 2: # INS
                    INS_msg.append(line)
                elif int(sparse_data[5]) == 3: # IMU
                    IMU_msg.append(line)
                line_count += 1
            if line_count < 3:
                # print (line)  # string
                line_count += 1

    return PVT_msg, INS_msg, IMU_msg

def sparse_PVT_msg(PVT_msg):
    timestamps = []
    # latitude = []
    # longitude = []
    easting = []
    northing = []
    altitude = []
    velocityEasting = []
    velocityNorthing = []
    velocityUp = []
    for i in range(len(PVT_msg)):
        tmp = PVT_msg[i].split(",")
        timestamps.append(np.float(tmp[0]))
        # 6 - 10  latitude, longitude, easting, northing, altitude,
        # 11 - 15 geoid, eastingStdDev, northingStdDev, altitudeStdDev, velocityEasting,
        # 16 - 20 velocityNorthing, velocityUp, solutionStatusOK, solutionStatus3D, solutionMode,
        # 21 - 25 solutionSource, numSatellites, numSatellitesPosition, numSatellitesVelocity, dGPSCorrectionAge,
        # 26 - 27 figureOfMerit, failureCode
        # latitude.append(np.float(tmp[6]))
        # longitude.append(np.float(tmp[7]))
        easting.append(np.float(tmp[8]))
        northing.append(np.float(tmp[9]))
        altitude.append(np.float(tmp[10]))
        velocityEasting.append(np.float(tmp[15]))
        velocityNorthing.append(np.float(tmp[16]))
        velocityUp.append(np.float(tmp[17]))

    return timestamps, easting, northing, altitude, velocityEasting, velocityNorthing, velocityUp

def plot_PVT_msg(timestamps_pvt, easting, northing, velocityEasting, velocityNorthing):
    plt.figure(1)
    plt.plot(easting, northing, 'g', label='gps')
    plt.plot(easting[0],northing[0], 'rx', label='start point')
    plt.plot(easting[-1],northing[-1], 'bx', label='end point')
    plt.title('GPS Trajectory in UTM coordinate')
    plt.grid(True)
    plt.xlabel('Easting [m]')
    plt.ylabel('Northing [m]')
    plt.legend(loc='upper right')

    plt.figure(2)
    plt.plot(timestamps_pvt, velocityNorthing, 'b', label='northing')
    plt.plot(timestamps_pvt, velocityEasting, 'r', label='easting')
    plt.title('Velocity of the cart')
    plt.grid(True)
    plt.xlabel('Timestamps [posix format]')
    plt.ylabel('Velocity [m/s]')
    plt.legend(loc='upper right')

    plt.show()

def sparse_INS_msg(INS_msg):
    timestamps = []
    roll = []
    pitch = []
    yaw = []
    rollRate = []
    pitchRate = []
    yawRate = []
    rollStdDev = []
    pitchStdDev = []
    yawStdDev = []
    for i in range(len(INS_msg)):
        tmp = INS_msg[i].split(",")
        timestamps.append(np.float(tmp[0]))
        # 6 - 10 insTime, status, roll, pitch, yaw,
        # 11 - 15 rollRate, pitchRate, yawRate, rollStdDev, pitchStdDev,
        # 16 - 18 yawStdDev, ekfCorrectionTime, ekfCorrectionDelay
        roll.append(np.float(tmp[8]))
        pitch.append(np.float(tmp[9]))
        yaw.append(np.float(tmp[10]))
        rollRate.append(np.float(tmp[11]))
        pitchRate.append(np.float(tmp[12]))
        yawRate.append(np.float(tmp[13]))
        rollStdDev.append(np.float(tmp[14]))
        pitchStdDev.append(np.float(tmp[15]))
        yawStdDev.append(np.float(tmp[16]))

    return timestamps, roll, pitch, yaw, rollRate, pitchRate, yawRate, rollStdDev, pitchStdDev, yawStdDev

def plot_INS_msg(timestamps_ins, roll, pitch, yaw, rollRate, pitchRate, yawRate):
    plt.figure(3)
    plt.subplot(311)
    plt.plot(timestamps_ins, roll, label='roll')
    plt.title('GPS angular measurement (body frame)')
    plt.ylabel('roll [rad]')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.subplot(312)
    plt.plot(timestamps_ins, pitch, label='pitch')
    plt.ylabel('pitch [rad]')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.subplot(313)
    plt.plot(timestamps_ins, yaw, label='yaw')
    plt.ylabel('yaw [rad]')
    plt.legend(loc='upper right')
    plt.xlabel('Timestamps [posix format]')
    plt.grid(True)

    plt.figure(4)
    plt.subplot(311)
    plt.plot(timestamps_ins, rollRate, label='rollRate')
    plt.title('GPS angular rate measurement (body frame)')
    plt.ylabel('rollRate [rad/s]')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.subplot(312)
    plt.plot(timestamps_ins, pitchRate, label='pitchRate')
    plt.ylabel('pitchRate [rad/s]')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.subplot(313)
    plt.plot(timestamps_ins, yawRate, label='yawRate')
    plt.ylabel('yawRate [rad/s]')
    plt.legend(loc='upper right')
    plt.xlabel('Timestamps [posix format]')
    plt.grid(True)

    plt.show()

def sparse_IMU_msg(IMU_msg):
    timestamps = []
    xAccel = []
    yAccel = []
    zAccel = []
    xGyroRate = []
    yGyroRate = []
    zGyroRate = []
    for i in range(len(IMU_msg)):
        tmp = IMU_msg[i].split(",")
        timestamps.append(np.float(tmp[0]))
        # 6 - 10 messageId,sequenceNumber,fractionOfInterval,xAccel,yAccel,
        # 11 - 15 zAccel,xGyroRate,yGyroRate,zGyroRate,xMag,
        # 16 - 20 yMag,zMag,xAccelTemp,yAccelTemp,zAccelTemp,
        # 21 - 24 xGyroTemp,yGyroTemp,zGyroTemp,error
        xAccel.append(np.float(tmp[9]))
        yAccel.append(np.float(tmp[10]))
        zAccel.append(np.float(tmp[11]))
        xGyroRate.append(np.float(tmp[12]))
        yGyroRate.append(np.float(tmp[13]))
        zGyroRate.append(np.float(tmp[14]))

    return timestamps, xAccel, yAccel, zAccel, xGyroRate, yGyroRate, zGyroRate

def plot_IMU_msg(timestamps_imu, xAccel, yAccel, zAccel, xGyroRate, yGyroRate, zGyroRate):
    plt.figure(5)
    plt.subplot(311)
    plt.plot(timestamps_imu, xAccel, label='xAccel')
    plt.title('IMU acceleration measurement (body frame)')
    plt.ylabel('a_x [m/s^2]')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.subplot(312)
    plt.plot(timestamps_imu, yAccel, label='yAccel')
    plt.ylabel('a_y [m/s^2]')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.subplot(313)
    plt.plot(timestamps_imu, zAccel, label='zAccel')
    plt.ylabel('a_z [m/s^2]')
    plt.legend(loc='upper right')
    plt.xlabel('Timestamps [posix format]')
    plt.grid(True)

    plt.figure(6)
    plt.subplot(311)
    plt.plot(timestamps_imu, xGyroRate, label='xGyroRate')
    plt.title('IMU gyroscope measurement (body frame)')
    plt.ylabel('xGyroRate [deg/s]')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.subplot(312)
    plt.plot(timestamps_imu, yGyroRate, label='yGyroRate')
    plt.ylabel('yGyroRate [deg/s]')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.subplot(313)
    plt.plot(timestamps_imu, zGyroRate, label='zGyroRate')
    plt.ylabel('zGyroRate [deg/s]')
    plt.legend(loc='upper right')
    plt.xlabel('Timestamps [posix format]')
    plt.grid(True)

    plt.show()


if __name__ == "__main__":
    csv_file_path = "/home/shu/Desktop/SF3000-recordings/2019_07_15_2047_17_perceptionLog_StructSensorSF3000.csv"

    PVT_msg, INS_msg, IMU_msg = read_gps_data(csv_file_path)
    # PVT (Position, Velocity, and Time in navigation systems, especially satellite-based)
    timestamps_pvt, easting, northing, altitude, velocityEasting, velocityNorthing, velocityUp = sparse_PVT_msg(PVT_msg)
    # INS
    timestamps_ins, roll, pitch, yaw, rollRate, pitchRate, yawRate, rollStdDev, pitchStdDev, yawStdDev = sparse_INS_msg(INS_msg)
    # IMU
    timestamps_imu, xAccel, yAccel, zAccel, xGyroRate, yGyroRate, zGyroRate = sparse_IMU_msg(IMU_msg)

    plot_PVT_msg(timestamps_pvt, easting, northing, velocityEasting, velocityNorthing)
    plot_INS_msg(timestamps_ins, roll, pitch, yaw, rollRate, pitchRate, yawRate)
    plot_IMU_msg(timestamps_imu, xAccel, yAccel, zAccel, xGyroRate, yGyroRate, zGyroRate)








