import numpy as np
import scipy as sp
from scipy.interpolate import RBFInterpolator
import matplotlib.pyplot as plt

def func(x, y):
    return x*(1-x)*np.cos(4*np.pi*x) * np.sin(4*np.pi*y**2)**2

def grid2points(x,y):
    return np.stack((x.ravel(), y.ravel()), -1)

def values2grid(points, shape):
    return np.reshape(points, shape)

def grid(xmin, xmax, nx, ymin, ymax, ny):
    return np.meshgrid(np.linspace(xmin, xmax, nx), np.linspace(ymin, ymax, ny), indexing='ij')

def interpolate(points, values, x, y, kernel):
    interpolator = RBFInterpolator(points, values, kernel=kernel)
    points = grid2points(x,y)
    values = interpolator(points)
    return values2grid(values, x.shape)

def plot(subplot, z, title):
    plt.subplot(subplot)
    plt.imshow(z.T, extent=(0, 1, 0, 1), origin='lower')
    plt.title(title)

def show():
    plt.show(block=False)

def pause():
    input('Press <ENTER> to continue')

def gradient(z, dx, dy):
    return np.gradient(z,dx,dy)
    

xmin = 0
xmax = 1
nx = 100
dx = (xmax-xmin)/(nx-1)

ymin = 0
ymax = 1
ny = 200
dy = (xmax-xmin)/(nx-1)

x, y = grid(xmin, xmax, nx, ymin, ymax, ny)

rng = np.random.default_rng()
points = rng.random((1000, 2))
values = func(points[:,0], points[:,1])

z = func(x, y)
z1 = interpolate(points, values, x, y, 'linear')
z2 = interpolate(points, values, x, y, 'thin_plate_spline')
z3 = interpolate(points, values, x, y, 'cubic')
z4 = interpolate(points, values, x, y, 'quintic')


gx, gy = gradient(z, dx, dy)
gx1, gy1 = gradient(z1, dx, dy)
gx2, gy2 = gradient(z2, dx, dy)
gx3, gy3 = gradient(z3, dx, dy)
gx4, gy4 = gradient(z4, dx, dy)

plt.figure()
plt.imshow(z.T, extent=(0, 1, 0, 1), origin='lower')
plt.plot(points[:, 0], points[:, 1], 'k.', ms=1)
plt.title('Original')
show()

plt.figure()
plot(221, z1, 'linear')
plot(222, z2, 'thin_plate_spline')
plot(223, z3, 'cubic')
plot(224, z4, 'quintic')
show()

pause()