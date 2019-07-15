
import os
for filename in os.listdir('.'):
	os.rename(filename, 'left-'+filename)

for filename in os.listdir("."):
	index = filename.split("_")[0]
	os.rename(filename, "IMG"+index+".png")

