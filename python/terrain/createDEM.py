import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
from scipy.interpolate import LinearNDInterpolator
from scipy.interpolate import NearestNDInterpolator
from pykrige.ok import OrdinaryKriging

def createDEM(x,y,z,w,h,fill_outside = False):
    # Create mesh
    points = np.column_stack((x, y))
    tri = Delaunay(points)
    # Interpolate
    linear_interpolator = LinearNDInterpolator(tri, z, fill_value=np.nan)
    xd, yd = np.meshgrid(np.linspace(xmin,xmax, w), np.linspace(ymin,ymax, h))
    zd = linear_interpolator(xd, yd)
    # Extraplate out of mesh
    if fill_outside:
        kriging_interpolator = OrdinaryKriging(x,y,z,variogram_model="spherical",verbose=False,enable_plotting=False)
        o = np.isnan(zd)
        zd[o], vark = kriging_interpolator.execute("points",xd[o], yd[o])
    return xd, yd, zd

def plotDEM(x,y,z):
    plt.pcolormesh(x,y,z, shading='auto')
    
xmin, xmax = -5, 5
ymin, ymax = -6, 6

# Points
n = 200
x = xmin + (xmax-xmin)*np.random.rand(n)
y = ymin + (ymax-ymin)*np.random.rand(n)
z = np.random.rand(n)

# DEM
xd, yd, zd = createDEM(x,y,z,2000,1000,True)

plotDEM(xd,yd,zd)
plt.plot(x, y, 'o', color='red')
plt.grid()
plt.show()