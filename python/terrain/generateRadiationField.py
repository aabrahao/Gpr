import Hydrology as hyd
import Geotiff as gt
import Dem as dm
import Visualization as vz
import numpy as np
import Stl as st

import matplotlib.pylab as plt


def rad(x,y,xp, yp, p):
    xx,yy = np.meshgrid(x,y)
    r = np.sqrt( np.power(xx-xp,2) + np.power(yy-yp,2) )
    i = p/(4*np.pi*(r*r))
    i[i>p] = p
    return i

def rand():
    return np.random.random()

def main():
 
    path = 'data/mesh/terrains/mars'
    view=(45,5)

    #path = 'data/mesh/part1'
    #view=(-45.0,5)

    database = gt.open( path )
    x,y,z = gt.dem(database)

    xs, ys = dm.xy(x,y,0.5,0.5)
    ps = 1
    i = rad(x,y,xs,ys,ps)

    n = 3
    for j in range(200):
        print(f'{j} ({j/n}%%)')
        xr = rand()
        yr = rand()
        p = 1*rand()
        xs, ys = dm.xy(x,y,xr,yr)
        i = i + rad(x,y,xs,ys,ps)

    vz.plotMesh(x,y,z,i,path + 'rad',view=view)
    dm.save(x,y,i,path + 'rad') 

if __name__ == "__main__":
    main()
