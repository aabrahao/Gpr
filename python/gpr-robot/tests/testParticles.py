import numpy as np
import matplotlib.pyplot as plt
from random import random 
import robot.visualizer.plotter.canvas as cnv
import robot.visualizer.plotter.particle as ptc

def main():

    canvas = cnv.Canvas((-10, 10, -10, 10))

    a = ptc.Particle(canvas, 0, 0)
    b = ptc.Particle(canvas, -10, -10) 

    t = 0
    tmax = 100
    dt = 0.1
    while (t <= tmax):
        a.move( t/10*np.cos(t), t/10*np.sin(t) )
        b.move( t/5-10, t/5-10 )
        canvas.update()
        t += dt
    canvas.show()
    
if __name__ == "__main__":
    main() 
