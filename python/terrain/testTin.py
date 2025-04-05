import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
from scipy.interpolate import LinearNDInterpolator
from scipy.interpolate import NearestNDInterpolator
from pykrige.ok import OrdinaryKriging

xmin, xmax = -5, 5
ymin, ymax = -6, 6

# Points
n = 200
x = xmin + (xmax-xmin)*np.random.rand(n)
y = ymin + (ymax-ymin)*np.random.rand(n)
z = np.random.rand(n)

plt.plot(x, y, 'o')
plt.grid()
plt.show(block=False)

# TIN
points = np.column_stack((x, y))
tri = Delaunay(points)
print(tri.points)
print(tri.simplices)
print(tri.neighbors)

plt.figure()
plt.triplot(tri.points[:,0], tri.points[:,1], tri.simplices)
plt.plot(tri.points[:,0], tri.points[:,1], 'o')
plt.grid()
plt.show(block=False)

# Interpolation
w = 2000
h = 1000
xm, ym = np.meshgrid(np.linspace(xmin,xmax, w), np.linspace(ymin,ymax, h))

# Linear
linear_interpolator = LinearNDInterpolator(tri, z, fill_value=np.nan)
zl = linear_interpolator(xm, ym)

plt.figure()
plt.title('Linear Interpotlation')
plt.pcolormesh(xm,ym,zl, shading='auto')
plt.plot(x, y, 'o', color='green')
plt.triplot(tri.points[:,0], tri.points[:,1], tri.simplices, color='red')
plt.grid()
plt.show(block=False)

# Nearest
nearest_interpolator = NearestNDInterpolator(points, z)
zn = nearest_interpolator(xm, ym)

plt.figure()
plt.title('Nearest Interpolation')
plt.pcolormesh(xm,ym,zn, shading='auto')
plt.plot(x, y, 'o', color='green')
plt.triplot(tri.points[:,0], tri.points[:,1], tri.simplices, color='red')
plt.grid()
plt.show(block=False)

# Kriging
kriging_interpolator = OrdinaryKriging(x,y,z,variogram_model="spherical",verbose=False,enable_plotting=False)
zk, vark = kriging_interpolator.execute("grid",np.linspace(xmin,xmax, w), np.linspace(ymin,ymax, h))

plt.figure()
plt.title('Kriging Interplation')
plt.pcolormesh(xm,ym,zk, shading='auto')
plt.plot(x, y, 'o', color='green')
plt.triplot(tri.points[:,0], tri.points[:,1], tri.simplices, color='red')
plt.grid()
plt.show()
