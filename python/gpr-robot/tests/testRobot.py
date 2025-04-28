import numpy as np
import robot.visualizer.plotter.canvas as cnv
import robot.visualizer.plotter.shapes as shp
import robot.simulation.robot as rb
import robot.optmizer.dubins as db

def feet(inches):
    return inches/12

def radians(degrees):
    return np.deg2rad(degrees)

def grid(x,y,w,h,s):
    x = []

def main():

    # Testing area
    wa = 30.0
    ha = 18.0

    # Robot
    rr = rb.turning_radius
    wr = rb.width()
    hr = rb.height()
    xi = 0.0
    yi = 0.0
    hi = radians(90)

    # Path
    ds = feet(2) # Point resolutin
    
    # Graphics
    canvas = cnv.Canvas((-0.5*wa-wr,0.5*wa+wr,-wr,ha+wr))
    workspace = shp.Rectangle(0,0.5*ha,wa,ha,color=(0.8,0.8,0.8),canvas=canvas)
    robot = rb.make(xi, yi, hi, canvas)
    
    # Path    
    planner = db.planner(radius=rr,point_separation=ds)
    x,y = db.path(planner,xi,yi,hi,0.5*wa,ha,radians(0))
    h = db.headings(x,y,hi)
    n = len(x)

    canvas.lines(x,y)
    for i in range(n):
        robot.move(x[i],y[i],h[i])
        canvas.update()
    
    canvas.show()
    
if __name__ == "__main__":
    main()