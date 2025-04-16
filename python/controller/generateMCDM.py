import Hydrology as hyd
import Geotiff as gt
import DEM as dm
import Visualization as vz
import numpy as np
import Stl as st
from scipy import stats as sts
import matplotlib.pylab as plt

import Numeric as nm

g_save_dem = True

def plot(x,y,z,c,title,view):
    vz.plotMesh(x,y,z,c,title,view=view) 

def decision(Z, cmin, cmax):
    Zd = Z.copy()
    Zd = nm.trim(Zd, cmin, cmax)
    Zd[np.isclose(Z,0.0)] = np.nan
    Zd = nm.norm(Zd)
    return Zd

def criteria(path, cmin, cmax):
    dataset = gt.open( path )
    x,y,Z = gt.dem(dataset)
    Z = decision(Z, cmin, cmax)
    return Z

def generate(path, view, cmin, cmax, w):
    # Dataset
    dataset = gt.open( path )
    x,y,Z = gt.dem(dataset)
    # Criteria
    Z1 = criteria(path + 'pit'    , cmin[0], cmax[0])
    Z2 = criteria(path + 'twi'    , cmin[1], cmax[1])
    Z3 = criteria(path + 'radmean', cmin[2], cmax[2])
    Z4 = criteria(path + 'radstd' , cmin[3], cmax[3])
    Z5 = criteria(path            , cmin[4], cmax[4])
    # MCDM
    Zd = w[0]*nm.nan(Z1, 0.0) + \
         w[1]*nm.nan(Z2, 0.0) + \
         w[2]*nm.nan(Z3, 0.0) + \
         w[3]*nm.nan(Z4, 0.0) + \
         w[4]*nm.nan(Z5, 0.0)
    # Normalization
    vz.disp(Zd)
    Zd = decision(Zd,0.0,1.0)
    vz.disp(Zd)
    # Action
    Za05 = decision(Zd, 0.05, 1.0)
    Za10 = decision(Zd, 0.10, 1.0)
    Za15 = decision(Zd, 0.15, 1.0)
    Za20 = decision(Zd, 0.20, 1.0)
    Za25 = decision(Zd, 0.25, 1.0)
    Za30 = decision(Zd, 0.30, 1.0)
    Za35 = decision(Zd, 0.35, 1.0)
    Za40 = decision(Zd, 0.40, 1.0)
    Za45 = decision(Zd, 0.45, 1.0)
    Za50 = decision(Zd, 0.50, 1.0)
    # Plot
    plot(x,y,Z,Z1, path + '-c1-pit' ,view)
    plot(x,y,Z,Z2, path + '-c2-twi' ,view)
    plot(x,y,Z,Z3, path + '-c3-radm',view)
    plot(x,y,Z,Z4, path + '-c4-rads',view)
    plot(x,y,Z,Z5, path + '-c5-hst' ,view)
    plot(x,y,Z,Zd, path + '-c0-dsc' ,view)
    plot(x,y,Z,Za05, path +  '-c0-act-05',view)
    plot(x,y,Z,Za10, path +  '-c0-act-10',view)
    plot(x,y,Z,Za15, path +  '-c0-act-15',view)
    plot(x,y,Z,Za20, path +  '-c0-act-20',view)
    plot(x,y,Z,Za25, path +  '-c0-act-25',view)
    plot(x,y,Z,Za30, path +  '-c0-act-30',view)
    plot(x,y,Z,Za35, path +  '-c0-act-35',view)
    plot(x,y,Z,Za40, path +  '-c0-act-40',view)
    plot(x,y,Z,Za45, path +  '-c0-act-45',view)
    plot(x,y,Z,Za50, path +  '-c0-act-50',view)
    # Save
    if g_save_dem:
        dm.save(x,y,  Z1, path + '-c1-pit')
        dm.save(x,y,  Z2, path + '-c2-twi')
        dm.save(x,y,  Z3, path + '-c3-radm')
        dm.save(x,y,  Z4, path + '-c4-rads')
        dm.save(x,y,  Z5, path + '-c4-hst')
        dm.save(x,y,  Zd, path + '-dsc')
        dm.save(x,y,Za05, path + '-act-05')
        dm.save(x,y,Za10, path + '-act-10')
        dm.save(x,y,Za20, path + '-act-20')
        dm.save(x,y,Za25, path + '-act-25')
        dm.save(x,y,Za30, path + '-act-30')
        dm.save(x,y,Za35, path + '-act-35')
        dm.save(x,y,Za40, path + '-act-40')
        dm.save(x,y,Za45, path + '-act-45')
        dm.save(x,y,Za50, path + '-act-50')

def main():
    vz.saveImages(True)
    generate( 'data/mesh/terrains/mars', 
              view=(45,5),
              cmin = (0.005, 1.4, 0.01, 0.01,  7.0),
              cmax = (None, None, None, None, 13.0),
              w = (1.0, 1.0, 1.0, 1.0, 1.0) )
    #generate( 'data/mesh/models/part1', 
    #          view=(-45.0,5),
    #          cmin = (0.005, 1.4, 0.9, 0.6),
    #          cmax = (None, None, None, 1.5),
    #          w = (0.0, 0.0, 0.0, 0.0) )

if __name__ == "__main__":
    main()
