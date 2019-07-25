import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib import cm


xs = np.array([.257733,.18611,.76414,.54115])
ys = np.array([.293,.9159,.334,.9358])
zs = np.array([1.078,.25756,.6411,.13245])
fit = np.array([-.604105,-1.233613,1.55607])

# plot raw data
plt.figure()
ax = plt.subplot(111, projection='3d')
ax.scatter(xs, ys, zs, color='b')

# plot plane
X = np.linspace(0,1,10)
Y = X
X,Y = np.meshgrid(X,Y)
Z = np.zeros(X.shape)
for r in range(X.shape[0]):
    for c in range(X.shape[1]):
        Z[r,c] = fit[0] * X[r,c] + fit[1] * Y[r,c] + fit[2]
ax.plot_wireframe(X,Y,Z, color='k')

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()
