import numpy as np
from scipy.interpolate import RegularGridInterpolator

import DEM as dm
import rasterio
from rasterio.transform import from_origin

import Geotiff as gt

import Numeric as nm

# DEM
# x: vector
# y: vector
# z: grid matrix

def noData():
    return -9999

def save(x,y,z,filename):
    dm.checkIndexing(x,y,z)
    nx = x.size
    ny = y.size
    xmin = x[0]
    ymin = y[0]
    xmax = x[-1]
    ymax = y[-1]
    dx = (xmax - xmin) / (nx - 1)
    dy = (ymax - ymin) / (ny - 1)
    transform = from_origin(xmin, ymax, dx, dy)
    database = rasterio.open( filename + '.tif',
                             'w',
                             driver = 'GTiff',
                             height = ny,
                             width = nx,
                             count = 1,
                             dtype = str(z.dtype),
                             crs='EPSG:32633',
                             transform = transform,
                             nodata=noData())
    database.write(z, 1)
    database.close()

def checkIndexing(x,y,z):
    ny, nx = z.shape
    if nx == x.size and ny == y.size:
        return True
    print(f'Dem indesing: x: {x.size}, y: {y.size}, z:[{nx},{ny}] > failed!')
    return False

def toPoints(x,y,z):
    xg, yg = grid 
    return np.column_stack((xg, yg, z))

def grid(x,y):
    return np.meshgrid(x,y,indexing='xy')

def points(x,y,mask=None):
    xg, yg = grid(x, y)
    xg = xg.flatten()
    yg = yg.flatten()
    if mask is not None:
        m = mask.flatten()
        i = np.where(np.isnan(m) | np.isclose(m,0.0))[0]
        return xg[i], yg[i]
    return xg, yg

def shape(x,y):
    nx = x.size
    ny = y.size
    return (ny, nx)

def size(x,y):
    nx, ny = shape(x,y)
    return nx*ny

def flatten(x,y,Z):
    X, Y = points(x,y)
    return X, Y, Z.flatten()

def unflatten(x,y,Z):
    return Z.reshape( shape(x,y) )

def xy(x,y,xp,yp): # xp, and yp [0, 1]
    xmin, ymin, xmax, ymax = dm.extents(x,y)
    xs = xmin + xp*(xmax - xmin)
    ys = ymin + yp*(ymax - ymin)
    return xs, ys

def sample(x,y,Z,n):
    i = nm.irand(0, Z.size-1, n)
    X,Y,ZS = flatten(x,y,Z)
    return X[i], Y[i], ZS[i]

def extents(x,y):
    xmin = np.nanmin(x)
    ymin = np.nanmin(y)
    xmax = np.nanmax(x)
    ymax = np.nanmax(y)
    return xmin, ymin, xmax, ymax

def lot(x,y):
    xmin,ymin,xmax,ymax = extents(x,y)
    return xmax-xmin, ymax-ymin

def resample(x,y,z,ds,method='linear'):
    xmin, ymin, xmax, ymax = extents(x,y)
    xs = np.arange(xmin, xmax, ds)
    ys = np.arange(ymin, ymax, ds)
    interpolator = RegularGridInterpolator((x, y), z.T, method=method, fill_value=noData())
    xg, yg = grid(xs,ys)
    zg = interpolator((xg,yg))
    return xs, ys, zg

def mask(z, nodata=noData()):
    m = np.where(np.isclose(z, noData()), 0.0, 1.0)
    return m

def shrink(x,y,z,border):
    xr = x[border:-border]
    yr = y[border:-border] 
    zr = z[border:-border, border:-border]
    checkIndexing(xr,yr,zr)
    return xr,yr,zr

####################################################

#class Dem:
#    def __init__(self,path):
#        dataset = gt.open(path+'.tif')
#        self.path = path
#        self.xmin, self.ymin, self.xmax, self.ymax = gt.extents(dataset)
#        self.z = gt.data(dataset)
#    def nx(self):
#        self.z.shape[1]
#    def ny(self):
#        self.z.shape[2]
#    def x(self):
#        return np.linspace(self.xmin, self.xmax, self.nx())
#    def y(self):
#        return np.linspace(self.ymin, self.ymax, self.ny())
#    def z(self):
#        return self.z
#    def xy(self):


#########################################################################
