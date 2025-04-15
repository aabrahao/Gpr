import Hydrology as hyd
import Geotiff as gt
import Dem as dm
import Visualization as vz
import numpy as np
import Stl as st
from scipy import stats as sts

import matplotlib.pylab as plt

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

def criteria(path, cmin, cmax=None):
    database = gt.open( path )
    x,y,z = gt.dem(database)
    z = trim(z, cmin, cmax)
    z = norm(z)
    return z

def plot(x,y,z,c,title,view):
    vz.plotMesh(x,y,z,c,title,view=view) 

def nan(v, a):
    x = v.copy()
    x[ np.isnan(x) ] = a
    return x
    
def main():

    path = 'data/mesh/terrains/mars'
    view=(45,5)

    database = gt.open( path )
    x,y,z = gt.dem(database)
    
    # Criteria
    z1 = criteria(path + 'pit', 0.005)
    z2 = criteria(path + 'twi', 2.0)
    z3 = criteria(path + 'rad', 0.0)
    z4 = criteria(path,         0.6, 1.5)
    
    vz.disp(z1)
    vz.disp(z2)
    vz.disp(z3)
    vz.disp(z4)

    # MCDM
    zm = nan(z1,0.0) + nan(z2, 0.0) + nan(z3, 0.0) + nan(z4,0.0)
    zm = norm(zm)
    zm[np.isclose(zm,0.0)] = np.nan
    vz.disp(zm)

    #plot(x,y,z,z1, path + '-c1-pit',view)
    #plot(x,y,z,z2, path + '-c2-twi',view)
    #plot(x,y,z,z3, path + '-c3-ang',view)
    #plot(x,y,z,z4, path + '-c5-hst',view)
    #plot(x,y,z,zm, path + '-cmi',view)

    dm.save(x,y,zm, path + '-cmi')

if __name__ == "__main__":
    main()
