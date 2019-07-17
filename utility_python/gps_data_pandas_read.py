import pandas
import matplotlib.pyplot as plt

df = pandas.read_csv('/home/shu/Downloads/JD/2019_06_26_ExtractedTUKLData_Log123/2019_06_26_1428_57_extractedPoseData_StructImageGroup.csv')

data = df[df['senID']==6]

plt.plot(data['x'], data['y'])
plt.show()