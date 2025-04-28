import numpy as np
from numpy import pi
import matplotlib.pylab as plt
import robot.visualizer.plotter.canvas as cnv
import robot.visualizer.plotter.shapes as shp
import robot.simulation.robot as rb
import robot.optmizer.dubins as db
import robot.visualizer.plotter.debugger as dbg

def main():

    # Testing area
    wa = 30.0
    ha = 18.0

    # Robot
    rr = rb.turning_radius
    wr = rb.width()
    hr = rb.height()

    # Start
    xs = 0.0
    ys = 0.0
    hs = np.deg2rad(90)

    # End
    rmin = 1*rb.turning_radius
    rmax = 4*rb.turning_radius
    
    # Graphscs

    canvas = cnv.Canvas((-1.2*rmax,1.2*rmax,-1.2*rmax,1.2*rmax))
    robot = rb.body(xs, ys, hs, canvas)
    
    canvas.line(0,0,rmax*np.cos( rb.steering_angle + hs),rmax*np.sin( rb.steering_angle + hs),color='blue',width=4)
    canvas.line(0,0,rmax*np.cos(-rb.steering_angle + hs),rmax*np.sin(-rb.steering_angle + hs),color='blue',width=4)

    planner = db.Planner(rb.turning_radius, 0.2)
    for r in np.linspace(rmax, rmin, 4):
        for a in np.linspace(0,2*pi,36):
            xe = r*np.cos(a)
            ye = r*np.sin(a)
            he = planner.optimal(xs,ys,hs,xe,ye)
            x,y,h = planner.path(xs,ys,hs,xe,ye,he)
            canvas.lines(x,y,color='black')
            canvas.point(xs,ys,color='black')
            canvas.point(xe,ye,color='black')
            canvas.update()

    canvas.line(0,0,rmax*np.cos(rb.steering_angle + hs),rmax*np.sin(rb.steering_angle + hs))

    canvas.show()
    
if __name__ == "__main__":
    main()