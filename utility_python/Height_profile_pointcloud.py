import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

pc_file = "/home/shu/Downloads/Rosario_2019/workstation_template_04/dense_geo/bk/fused_cropped_small.txt"

x = []
y = []
z = []
r = []
g = []
b = []
with open(pc_file, 'r') as fp:
    for line in fp:
        x.append(np.float(line.split(' ')[0]))
        y.append(np.float(line.split(' ')[1]))
        z.append(np.float(line.split(' ')[2]))
        r.append(np.float(line.split(' ')[3])/255.0)
        g.append(np.float(line.split(' ')[4])/255.0)
        b.append(np.float(line.split(' ')[5])/255.0)

# r = np.asarray(r)
# g = np.asarray(g)
# b = np.asarray(b)

color_ = []
for i in range(len(r)):
    color_.append(np.array([r[i],g[i],b[i]]))



fig = plt.figure()
ax = fig.gca(projection='3d')

# for i in range(len(x)):
ax.scatter(x, y, z, marker='.', c = color_)

plt.legend(loc='best')
# plt.grid(True)
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
# ax.set_title('Geometric profile of the crop)
ax.autoscale()

plt.show()

