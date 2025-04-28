import robot.optmizer.dubins as db
import robot.visualizer.plotter.canvas as cv
import matplotlib.pyplot as plt
from robot.visualizer.benchmark import benchmark
from robot.visualizer.benchmark import error
import robot.utilities.numerics as nm
import numpy as np

def compare(planner,xs,ys,hs,xe,ye,he):
    x,y = db.path(planner,xs,ys,hs,xe,ye,he)
    l = benchmark(db.length, planner,xs,ys,hs,xe,ye,he)
    lp =  benchmark(db.length_points, planner,xs,ys,hs,xe,ye,he)
    plt.plot(x,y)
    return x,y,l,lp

def main():
    planner = db.planner(radius = 2.0, point_separation = 0.1)

    n = 5
    x = 100*nm.random(n)
    y = 100*nm.random(n)
    h = 2*np.pi*nm.random(n)

    e = []
    plt.scatter(x,y)
    plt.grid(True)
    plt.axis("equal")
        
    for i in range(n-1):
        x,y,l,lp = compare(planner,x[i],y[i],h[i],x[i+1],y[i+1],h[i+1])
        e.append(l-lp)

    plt.show()
    
    plt.plot(e)
    plt.show()
        
 
if __name__ == "__main__":
    main()