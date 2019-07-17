import os

img_list = os.listdir("./Sensor_5/slot_1/")

with open("./timestamps.txt", "w") as fp:
	for name in img_list:
		ts = name.split("_")[-1].split(".")[0] + name.split("_")[-1].split(".")[1]
		
		fp.write(ts + '\n')