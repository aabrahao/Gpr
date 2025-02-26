import rasterio
from rasterio import open
from rasterio.plot import show
import Print as prt

def extents(dataset):
    xmin = dataset.bounds.left
    xmax = dataset.bounds.right
    ymin = dataset.bounds.bottom
    ymax = dataset.bounds.top
    return xmin, ymin, xmax, ymax

def bounds(dataset):
    x, y = origin(dataset)
    w, h = width(dataset)
    return x, y, w, h

def size(dataset):
    iw = dataset.height
    jw = dataset.width
    return iw, jw

def width(dataset):
    xmin, ymin, xmax, ymax = extents(dataset)
    return xmax - xmin, ymax - ymin

def origin(dataset):
    xmin, ymin, xmax, ymax = extents(dataset)
    return xmin, ymin

def resolution(dataset):
    sx = dataset.transform[0]
    sy = -dataset.transform[4]
    return sx, sy

def info(dataset):
    prt.section('Dataset')
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