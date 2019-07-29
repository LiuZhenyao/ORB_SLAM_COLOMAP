import pandas
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

df = pandas.read_csv('/home/shu/Downloads/JD/2019_06_26_ExtractedTUKLData_Log123/2019_06_26_1428_57_extractedPoseData_StructImageGroup.csv')

data = df[df['senID']==6]

plt.figure()
plt.plot(data['x'], data['y'])
plt.title('GPS trajectory')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.grid(True)


fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(data['x'], data['y'], data['z'], label='GPS')
ax.legend()
# ax.autoscale()

plt.show()