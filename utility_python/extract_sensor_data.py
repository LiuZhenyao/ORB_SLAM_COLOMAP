import os
import shutil

# extracting zip file "2019_06_26_ExtractedTUKLData_Log123.zip"
# then, separate data into different folder

def move_file(path, sensor_type, slot_type):
	if slot_type == "0":
		savepath = os.path.join(path, "slot_"+slot_type)
		shutil.move(os.path.join(data_path, name), os.path.join(savepath, name))

	elif slot_type == "1":
		savepath = os.path.join(path, "slot_"+slot_type)
		shutil.move(os.path.join(data_path, name), os.path.join(savepath, name))

	elif slot_type == "2":
		savepath = os.path.join(path, "slot_"+slot_type)
		shutil.move(os.path.join(data_path, name), os.path.join(savepath, name))

	elif slot_type == "3":
		savepath = os.path.join(path, "slot_"+slot_type)
		shutil.move(os.path.join(data_path, name), os.path.join(savepath, name))


if __name__ == "__main__":
	data_path = "./2019_06_26_ExtractedTUKLData_Log123"
	data_list = os.listdir(data_path)
	for name in data_list:
		sensor_type = name.split("_")[2]
		slot_type = name.split("_")[4]

		if sensor_type == "5":
			path = "./Sensor_5"
			move_file(path, sensor_type, slot_type)


		elif sensor_type == "6":
			path = "./Sensor_6"
			move_file(path, sensor_type, slot_type)
