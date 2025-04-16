import numpy as np
import matplotlib.pyplot as plt
from random import random 
import Canvas as cv
import Particle as pt
import Geotiff as gt
import DEM as dm
import Visualization as vz

def main():
    a = pt.Particle(0, 0)
    t = 0
    tmax = 100
    dt = 0.1
    while (t <= tmax):
        xp,yp = a.position()
        a.move( xn, yn )
        plt.pause(.01)
        t += dt
   
if __name__ == "__main__":
    main() 
