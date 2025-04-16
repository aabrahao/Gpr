import numpy as np
from stl import mesh
import pyvista as pv

import DEM as dem
import Visualization as vz

from scipy.spatial import Delaunay
from scipy.interpolate import LinearNDInterpolator

# Pointcould
# x: scatter vector
# y: scatter vector
# z: scatter vector

def extents(x,y,z):
    xmin = np.nanmin(x.ravel())
    ymin = np.nanmin(y.ravel())
    zmin = np.nanmin(z.ravel())
    xmax = np.nanmax(x.ravel())
    ymax = np.nanmax(y.ravel())
    zmax = np.nanmax(z.ravel())
    return xmin, ymin, zmin, xmax, ymax, zmax

def toPoints(x,y,z):
    return np.column_stack((x, y, z))

def fromPoints(points):
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    return x,y,z

def unique(x,y,z):
    points = toPoints(x,y,z)
    points = np.unique(points, axis=0)
    return fromPoints(points)

def resample(x,y,z,ds,method='linear'):
    xu, yu, zu = unique(x,y,z)
    xmin = np.min(xu)
    ymin = np.min(yu)
    xmax = np.max(xu)
    ymax = np.max(yu)
    xs = np.arange(xmin, xmax, ds)
    ys = np.arange(ymin, ymax, ds)
    # Meshing
    points = np.column_stack((xu, yu))
    mesh = Delaunay(points)
    # Interpolate
    interpolator = LinearNDInterpolator(mesh, zu, fill_value=np.nan)
    xd, yd = dem.grid(xs, ys)
    zs = interpolator(xd, yd)
    return xs, ys, zs