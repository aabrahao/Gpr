import numpy as np
import matplotlib.pyplot as plt
from random import random 
import Canvas as cv
import Particle as pt

def rand(rmin=0.0, rmax=1.0):
    return (rmax-rmin)*random() + rmin

def main():
    canvas = cv.Canvas(-10, -10, 10, 10)
    a = pt.Particle(0, 0)
    b = pt.Particle(-10, -10) 

    t = 0
    tmax = 100
    dt = 0.1
    while (t <= tmax):
        a.move( t/10*np.cos(t), t/10*np.sin(t) )
        b.move( t/5-10, t/5-10 )
        canvas.update(t)
        t += dt

    canvas.show()
    
if __name__ == "__main__":
    main() 
