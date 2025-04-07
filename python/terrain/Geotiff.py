import rasterio
from rasterio import open as ropen
import Print as prt

import numpy as np
from rasterio.plot import show as rshow
import matplotlib.pylab as plt
from matplotlib import cbook, cm

# Rasterio and nunpy
# +-- column (x)
# |     .(y, x)
# row (y)
#
# (rows, columns) -> (y, x) : (height, width)
#
# Numpy is row-major order, elements are stored row by row
# (i,j) -> (y,x)

def filename(path):
    return path + '.tif'

def open(path):
    return ropen( filename(path) )

def mask(database, band = 1):
    mask = database.read_masks(band)

def name(database):
    s = database.name.rsplit('/', 1)[-1]
    s = s.replace('.tif','')
    return s

def data(database, band = 1):
    z = database.read(band)
    if np.issubdtype(z.dtype, np.floating):
        m = database.read_masks(band)
        z[m==0] = np.nan
    return z

def dem(database, band = 1):
    xmin, ymin, xmax, ymax = extents(database)
    z = data(database, band)
    nx, ny = size(database)
    x = np.linspace( xmin, xmax, nx)
    y = np.linspace( ymin, ymax, ny)
    return x, y, z

def show(database, ax=None, colormap = cm.gist_earth, block=True):
    if not ax:
        fig, axes = plt.subplots()
        axes.set_title(database.name)
    else:
        axes = ax
    rshow(database, ax=axes)
    if not ax:
        plt.show(block=block)
    return ax

def pause():
    plt.show(block=True)

def extents(database):
    xmin = database.bounds.left
    xmax = database.bounds.right
    ymin = database.bounds.bottom
    ymax = database.bounds.top
    return xmin, ymin, xmax, ymax

def size(database):
    ny = database.height
    nx = database.width
    return nx, ny

def origin(database):
    xmin, ymin, xmax, ymax = extents(database)
    return xmin, ymin

def resolution(database):
    sx = database.transform[0]
    sy = -database.transform[4]
    return sx, sy

def info(database):
    prt.section('database')
    prt.field('File', database.name)
    prt.field('Driver', database.driver)
    prt.field('Mode', database.mode)
    prt.field('Size', size(database))
    prt.field('Bands', database.count)
    prt.field('Indexes', database.indexes)
    prt.field('Types', database.dtypes)
    prt.field('Nodata', database.nodata)
    prt.field('Coordnates', database.crs)
    prt.section('Geometry')
    prt.field('Units', database.crs.linear_units)
    prt.field('Extents', extents(database))
    prt.field('Resolution', resolution(database))
    prt.field('Origin', origin(database))
    #prt.field('Width', width(database))
    prt.section('Transform')
    prt.field(database.transform)
    prt.section('Profile')
    prt.json(database.profile)
    prt.section()