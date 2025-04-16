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

def mask(dataset, band = 1):
    mask = dataset.read_masks(band)

def name(dataset):
    s = dataset.name.rsplit('/', 1)[-1]
    s = s.replace('.tif','')
    return s

def data(dataset, band = 1):
    z = dataset.read(band)
    if np.issubdtype(z.dtype, np.floating):
        m = dataset.read_masks(band)
        z[m==0] = np.nan
    return z

def dem(dataset, band = 1):
    xmin, ymin, xmax, ymax = extents(dataset)
    z = data(dataset, band)
    nx, ny = size(dataset)
    x = np.linspace( xmin, xmax, nx)
    y = np.linspace( ymin, ymax, ny)
    return x, y, z

def show(dataset, ax=None, colormap = cm.gist_earth, block=True):
    if not ax:
        fig, axes = plt.subplots()
        axes.set_title(dataset.name)
    else:
        axes = ax
    rshow(dataset, ax=axes)
    if not ax:
        plt.show(block=block)
    return ax

def pause():
    plt.show(block=True)

def extents(dataset):
    xmin = dataset.bounds.left
    xmax = dataset.bounds.right
    ymin = dataset.bounds.bottom
    ymax = dataset.bounds.top
    return xmin, ymin, xmax, ymax

def size(dataset):
    ny = dataset.height
    nx = dataset.width
    return nx, ny

def origin(dataset):
    xmin, ymin, xmax, ymax = extents(dataset)
    return xmin, ymin

def width(dataset):
    xmin, ymin, xmax, ymax = extents(dataset)
    return xmax-xmin, ymax-ymin

def resolution(dataset):
    sx = dataset.transform[0]
    sy = -dataset.transform[4]
    return sx, sy

def info(dataset):
    prt.section('dataset')
    prt.field('File', dataset.name)
    prt.field('Driver', dataset.driver)
    prt.field('Mode', dataset.mode)
    prt.field('Size', size(dataset))
    prt.field('Bands', dataset.count)
    prt.field('Indexes', dataset.indexes)
    prt.field('Types', dataset.dtypes)
    prt.field('Nodata', dataset.nodata)
    prt.field('Coordnates', dataset.crs)
    prt.section('Geometry')
    prt.field('Units', dataset.crs.linear_units)
    prt.field('Extents', extents(dataset))
    prt.field('Resolution', resolution(dataset))
    prt.field('Origin', origin(dataset))
    prt.field('Width', width(dataset))
    prt.section('Transform')
    prt.field(dataset.transform)
    prt.section('Profile')
    prt.json(dataset.profile)
    prt.section()