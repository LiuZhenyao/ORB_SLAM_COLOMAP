import pandas
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

df = pandas.read_csv('/home/shu/Downloads/JD/2019_06_26_ExtractedTUKLData_Log123/2019_06_26_1428_57_extractedPoseData_StructImageGroup.csv')

data = df[df['senID']==5]



x = data['x'].tolist()
y = data['y'].tolist()

X = [item - x[0] for item in x]
Y = [item - y[0] for item in y]

d = np.sqrt((X[0]-X[-1])*(X[0]-X[-1]) + (Y[0]-Y[-1])*(Y[0]-Y[-1]))



plt.figure()
plt.plot(X, Y, label='sensor 5, slot 0')
plt.title('Trajectory of sensor 5, slot 0 (local frame)')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend()
plt.grid(True)


# fig = plt.figure()
# ax = fig.gca(projection='3d')
# ax.plot(data['x'], data['y'], data['z'], label='GPS')
# ax.legend()
# # ax.autoscale()

plt.show()