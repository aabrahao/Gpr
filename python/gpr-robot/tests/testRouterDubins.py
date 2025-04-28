import numpy as np
import robot.utilities.numerics as nm
import robot.optmizer.router as rt
from robot.visualizer.benchmark import benchmark
import robot.visualizer.plotter.canvas as cv

def main():
    n = 20
    x = 100*nm.random(n)
    y = 100*nm.random(n)
    
    canvas = cv.Canvas(cv.limits(x,y))
    canvas.points(x,y)

    nodes = benchmark(rt.solve,x,y,open_ended=open_ended)

    rt.display(x, y, nodes)
    x,y = rt.route(x,y,nodes)
    canvas.lines(x,y)
    return x,y

    x1, y1 = test(x,y,True,canvas)
    x2, y2 = test(x,y,False,canvas)
        
    canvas.show()
    
if __name__ == "__main__":
    main()