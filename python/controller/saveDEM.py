import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LightSource
from mpl_toolkits.mplot3d import Axes3D

import rasterio
from rasterio.transform import from_origin

def saveDem(x,y,z,filename):
    nx = len(x)
    ny = len(y)
    xmin, xmax = x[0], x[-1]
    ymin, ymax = y[0], y[-1]
    dx = (xmax - xmin) / (nx - 1)
    dy = (ymax - ymin) / (ny - 1)
    
    transform = from_origin(xmin, ymax, dx, dy)

    dataset = rasterio.open( filename + '.tif',
                            'w',
                            driver = 'GTiff',
                            height = nx,
                            width = ny,
                            count = 1,
                            dtype = str(z.dtype),
                            crs='EPSG:32633',
                            transform = transform)
    dataset.write(z, 1)
    dataset.close()

def plotDem(x,y,z,block=False):
    plt.figure()
    plt.pcolormesh(x, y, z, cmap='jet', shading='auto')
    plt.axis('scaled')
    plt.colorbar()
    plt.show(block=block)

def set3dAxesEqual(ax):
    # Limits
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()
    # Ranges
    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])
    max_range = max(x_range, y_range, z_range)
    # Mid
    x_mid = np.mean(x_limits)
    y_mid = np.mean(y_limits)
    z_mid = np.mean(z_limits)
    # Set each axis so all have the same range centered at their midpoints
    ax.set_xlim3d([x_mid - max_range/2, x_mid + max_range/2])
    ax.set_ylim3d([y_mid - max_range/2, y_mid + max_range/2])
    ax.set_zlim3d([z_mid - max_range/2, z_mid + max_range/2])

def plotDem3D(x,y,z,block=False):
    xx, yy = np.meshgrid(xr, yr)    
    fig = plt.figure()
    ax = plt.axes(projection ='3d')
    surf = ax.plot_surface(xx, yy, z, cmap='jet', rstride=1, cstride=1, linewidth=0, edgecolor='none', antialiased=False, shade=True )
    fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10, label='Elevation')
    ax.view_init(elev=30, azim=100)
    set3dAxesEqual(ax)
    plt.show(block=block)

xr = np.linspace(-6, 6, 200)
yr = np.linspace(-5, 5, 200)

# full coordinate arrays
x, y = np.meshgrid(xr, yr)
#z = np.sqrt(x**2 + y**2)
z = np.random.rand(len(xr), len(yr))

saveDem(xr,yr,z, 'sphere')
plotDem(xr,yr,z,False)
plotDem3D(xr,yr,z,True)
