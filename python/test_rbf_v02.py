import numpy as np
import scipy as sp
from scipy.interpolate import RBFInterpolator
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.colors import LightSource

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

def plot(ax, x, y, z, title):
    ls = LightSource(azdeg=0, altdeg=65)
    rgb = ls.shade(z, plt.cm.RdYlBu)
    ax.plot_surface(x,y,z, cmap=cm.jet, rstride=1, cstride=1, linewidth=0,
                       antialiased=False, facecolors=rgb)
    ax.view_init(90,-90,0)
    plt.title(title)

def show():
    plt.show(block=False)

def pause():
    input('Press <ENTER> to continue')

def gradient(z, dx, dy):
    return np.gradient(z,dx,dy)

xmin = 0
xmax = 1
nx = 300
dx = (xmax-xmin)/(nx-1)

ymin = 0
ymax = 1
ny = 300
dy = (xmax-xmin)/(nx-1)

x, y = grid(xmin, xmax, nx, ymin, ymax, ny)

rng = np.random.default_rng()
points = rng.random((1000, 2))
values = func(points[:,0], points[:,1])

z1 = func(x, y)
z2 = interpolate(points, values, x, y, 'thin_plate_spline')
z3 = interpolate(points, values, x, y, 'cubic')
z4 = interpolate(points, values, x, y, 'quintic')

fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, subplot_kw={"projection": "3d"})
plot(ax1, x,  y, z1, 'Original')
plot(ax2, x,  y, z2, 'thin_plate_spline')
plot(ax3, x,  y, z3, 'cubic')
plot(ax4, x,  y, z4, 'quintic')
show()

#gx1, gy1 = gradient(z1, dx, dy)
#gx2, gy2 = gradient(z2, dx, dy)
#gx3, gy3 = gradient(z3, dx, dy)
#gx4, gy4 = gradient(z4, dx, dy)

#print(gx1.shape)

#plt.figure()
#plt.quiver(gx1, gy1)
#show()

pause()