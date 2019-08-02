import os
import numpy as np
import cv2

keyframe_file_path = '/home/shu/OpensourceProject/MYNT-ORBSLAM2_ws/src/MYNT-EYE-ORB-SLAM2-Sample/pose_graph/KeyFrameTrajectory.txt'

timestemps = []

with open(keyframe_file_path, 'r') as fp:
    for line in fp:
        # print (line)
        tmp = line.split(' ')[0]
        # print (tmp)
        tmp2 = tmp.split('.')[0] + tmp.split('.')[1]
        tmp3 = tmp2 + '0000'
        timestemps.append(tmp3)
        # print (timestemps)

# img_file_path = '/home/shu/Downloads/JD/2019_06_26_ExtractedTUKLData_Log123/Sensor_5/orb_slot_0'
img_file_path = '/home/shu/kalibr_workspace/output_vins_data2/cam0'
img_list = os.listdir(img_file_path)
for name in img_list:
    for ts in timestemps:
        if name.split('.')[0] == ts:
            img = cv2.imread(os.path.join(img_file_path, name))
            cv2.imwrite(os.path.join('/home/shu/Downloads/COLMAP_TEST/testfromORBSLAM2/imgs', name), img)

os.chdir('/home/shu/Downloads/COLMAP_TEST/testfromORBSLAM2')
os.system('python change_img_name.py')