import numpy as np
from numpy import pi
import matplotlib.pylab as plt
import robot.visualizer.plotter.canvas as cnv
import robot.visualizer.plotter.shapes as shp
import robot.simulation.robot as rb
import robot.optmizer.dubins as db
import robot.visualizer.plotter.debugger as dbg

def coverGrid(w,h,ra,canvas):
    x = np.arange(-0.5*w+ra,0.5*w+ra,2*ra)
    y = np.arange(ra,h,2*ra)
    (xg,yg) = plt.meshgrid(x,y)
    print(xg.shape)
    xg = xg.flatten()
    yg = yg.flatten()
    canvas.circles(xg,yg,ra,color='blue',thickness=2,fill=True)
    canvas.points(xg,yg,labels='indices',color='black',size=10,alpha=1)

def grid(w,h,ra,canvas):
    nx = int(w/(2*ra))
    ny = int(h/(2*ra))
    print(nx,ny)

def main():

    # Testing area
    w = rb.area_with
    h = rb.area_height
    
    # Antenna
    ra = rb.antenna_radious

    # Start
    xs = rb.robot_x
    ys = rb.robot_y
    hs = rb.robot_heading
    
    # Graphiscs
    canvas = cnv.Canvas(rb.limits())
    area = rb.area(canvas)
    robot = rb.body(xs, ys, hs, canvas,color='red',fill=True)
    canvas.point(0.0,0.0,color='red',size=10,alpha=1)
    
    coverGrid(w,h,ra,canvas)
    grid(w,h,ra,canvas)

    #canvas.line(0,0,rmax*np.cos( rb.steering_angle + hs),rmax*np.sin( rb.steering_angle + hs),color='blue',width=4)
    #canvas.line(0,0,rmax*np.cos(-rb.steering_angle + hs),rmax*np.sin(-rb.steering_angle + hs),color='blue',width=4)

    #planner = db.Planner(rb.turning_radius, 0.2)
    #for r in np.linspace(rmax, rmin, 4):
    #     for a in np.linspace(0,2*pi,36):
    #         xe = r*np.cos(a)
    #         ye = r*np.sin(a)
    #         he = planner.optimal(xs,ys,hs,xe,ye)
    #         x,y,h = planner.path(xs,ys,hs,xe,ye,he)
    #         canvas.lines(x,y,color='black')
    #         canvas.point(xs,ys,color='black')
    #         canvas.point(xe,ye,color='black')
    #         canvas.update()

    # canvas.line(0,0,rmax*np.cos(rb.steering_angle + hs),rmax*np.sin(rb.steering_angle + hs))

    canvas.show()
    
if __name__ == "__main__":
    main()