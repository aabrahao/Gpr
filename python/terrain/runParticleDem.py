import numpy as np
import matplotlib.pyplot as plt
from random import random 
import Canvas as cv
import Particle as pt
import Geotiff as gt
import Dem as dm
import Visualization as vz

def imin(x):
    i = np.argmin(x)
    if i.size > 1:
        i = i[0]
    return i

def rand(rmin=0.0, rmax=1.0):
    return (rmax-rmin)*random() + rmin

def distance(x1,y1,x2,y2):
    return np.sqrt( np.power(x2-x1, 2) + np.power(y2-y1, 2) )

def roi():
    plt.figure()
    ax = plt.gca()
    
    path = 'data/mesh/terrains/mars'
    database = gt.open(path + '-cmi')
    x,y,z = gt.dem(database)
    x,y = dm.grid(x,y)

    #ax = gt.show(database, ax) 
    
    x = x.flatten()
    y = y.flatten()
    z = z.flatten()

    nan = np.isnan(z)
    x = np.delete(x, nan)
    y = np.delete(y, nan)
    z = np.delete(z, nan)

    plt.plot(x,y,'.b')

    return x,y,z

def near(x,y,z,xn,yn):
    r = distance(x,y,xn,yn)
    i = imin(r)
    return x[i],y[i],z[i]

def extractNear(x,y,z,xn,yn):
    r = distance(x,y,xn,yn)
    i = imin(r)
    xn = x[i]
    yn = y[i]
    zn = z[i]
    x = np.delete(x,i)
    y = np.delete(y,i)
    z = np.delete(z,i)
    return x,y,z,xn,yn,zn

def main():

    x,y,z = roi()
   
    a = pt.Particle(0, 0)
    
    t = 0
    tmax = 100
    dt = 0.1
    while (t <= tmax):
        xp,yp = a.position()

        x,y,z,xn,yn,zn = extractNear(x,y,z,xp,yp)

        a.move( xn, yn )
        plt.pause(.01)
        t += dt

    canvas.show()
    
if __name__ == "__main__":
    main() 
