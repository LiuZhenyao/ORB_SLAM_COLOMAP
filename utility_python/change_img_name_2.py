import os

img_list = os.listdir("/home/shu/Downloads/COLMAP_TEST/testfromORBSLAM2/imgs")
img_list.sort()
index_num = 1

for filename in img_list:
	tmp = ""
	if index_num < 10:
		tmp = "00000" + str(index_num)
	elif index_num > 9 and index_num < 100:
		tmp = "0000" + str(index_num)
	elif index_num > 99 and index_num < 1000:
		tmp = "000" + str(index_num)
	elif index_num > 999 and index_num < 10000:
		tmp = "00" + str(index_num)
	elif index_num > 9999 and index_num < 100000:
		tmp = "0" + str(index_num)
	os.rename("./imgs/"+filename, "./imgs/IMG"+str(tmp)+".png")
    index_num += 1

