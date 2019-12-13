import os
import cv2

# ORB-SLAM + COLMAP:
#   as long as ORB-SLAM save the pose graph as KeyFrameTrajectory.txt, we extract corresponding keyframes

keyframe_file_path = '/home/shu/dense_orbslam_ws/ORB_SLAM2_REMODE/backup_trajectory/KeyFrameTrajectory_04.txt'
timestemps = []

with open(keyframe_file_path, 'r') as fp:
    for line in fp:
        tmp = line.split(' ')[0]
        timestemps.append(tmp)


img_file_path = '/home/shu/Downloads/Rosario_2019/dataset/04/sequence04/zed'
img_list = os.listdir(img_file_path)
for name in img_list:
    for ts in timestemps:
        if name.split('_')[0] == 'left' and name.split('_')[1] == (ts+".png"):
            img = cv2.imread(os.path.join(img_file_path, name))
            cv2.imwrite(os.path.join(
                '/home/shu/Downloads/Rosario_2019/workstation_template_04/imgs', name.split('_')[1]), img)

# os.chdir('/home/shu/Downloads/COLMAP_TEST/testfromORBSLAM2')
# os.system('python change_img_name.py')
