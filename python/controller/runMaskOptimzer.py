import numpy as np
import matplotlib.pyplot as plt
from random import random 
import Canvas as cv
import Particle as pt
import Geotiff as gt
import DEM as dm
import Visualization as vz

import GraphOptimizer as go

def norm(x):
    xmin = np.nanmin(x)
    xmax = np.nanmax(x)
    return (x - xmin) / (xmax - xmin)

def trim(v,vmin, vmax=None):
    x = v.copy()
    x[x < vmin] = np.nan
    if vmax is not None:
        x[x > vmax] = np.nan
    return x

def roi():
    path = 'data/mesh/terrains/mars'
    database = gt.open(path + '-cmi')
    x,y,z = gt.dem(database)
    # Start and end
    xs, ys = dm.xy(x,y,0.5,0.0)
    print(f'Start: ({xs},{ys})')
    xe, ye = dm.xy(x,y,0.5,1.0)
    print(f'End: ({xe},{ye})')
    # Grid
    x,y = dm.grid(x,y)
    # Points
    x = x.flatten()
    y = y.flatten()
    z = z.flatten()
    # Trim
    z[ z < 0.6 ] = np.nan
    # Remove nodata
    nan = np.isnan(z)
    x = np.delete(x, nan)
    y = np.delete(y, nan)
    z = np.delete(z, nan)
    return x,y,z,xs,ys,xe,ye

def main():
    x,y,z,xs,ys,xe,ye = roi()
    plt.plot(x,y,'.b')

    xr,yr = go.optimize(x,y,xs,ys,xe,ye)
    print("###")
    print(xr)
    print(yr)
    
    a = pt.Particle(xr[0], yr[0])

    for i in range( len(xr) ):
        a.move( xr[i], yr[i] )
        plt.pause(.1)     

    plt.show()
    
if __name__ == "__main__":
    main() 
