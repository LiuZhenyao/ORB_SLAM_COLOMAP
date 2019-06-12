### Intrinsic calibration 

 

##### Record data for calibration, print 9x6 openCV chessboard first  http://docs.opencv.org/master/pattern.png

cd /home/shu/MYNT-EYE-S-SDK 

source wrappers/ros/devel/setup.bash 

./tools/_output/bin/dataset/record2 

 

##### Use the tool in the VINS-Mono to calibrate 

cd /home/shu/catkin_ws/src/VINS-Mono/camera_model/calibration_test 

 

##### Add "left-" to every image you have 

cd left-cali_date 

ipython 

import os 

for filename in os.listdir('.'): 
  os.rename(filename, 'left-'+filename) 

cd .. 

 

##### Run calibration 

rosrun camera_model Calibration -w 9 -h 6 -s 23 -i left-cali_date --camera-model pinhole 

 

##### Write calibration result into config file 

/home/shu/catkin_ws/src/VINS-Mono/config/mynteye/mynteye_s_config.yaml  

and maybe device_params_left.yaml 

 

##### Remember to modify the file mynteye_s_config.yaml: 

Using the corrected image on the fly, image_topic: "/mynteye/left_rect/image_rect" 

 

 

##### (Optional) Maybe write parameters into the device after calibration? 

cd <sdk> 

./tools/_output/bin/writer/img_params_writer tools/writer/config/S1030/img.params 
