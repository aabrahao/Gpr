import numpy as np
from numpy import pi
import matplotlib.pylab as plt
import robot.visualizer.plotter.canvas as cnv
import robot.visualizer.plotter.shapes as shp
import robot.simulation.robot as rb
import robot.optmizer.dubins as db
import robot.visualizer.plotter.debugger as dbg

def play(planner,xs,ys,hs,xe,ye,canvas=None):
    he = []
    le = []
    for a in np.linspace(0,2*np.pi,int(360/5)):
        x,y,h = planner.path(xs,ys,hs,xe,ye,a)
        l = planner.length(xs,ys,hs,xe,ye,a)
        le.append(l)
        he.append(h[-1])
        #if canvas:
        #    canvas.lines(x,y,color='gray')
    he = np.array(he)
    le = np.array(le)
    i = np.argmin(le)
    he = he[i]
    le = le[i]
    if canvas:
        x,y,h = planner.path(xs,ys,hs,xe,ye,he)
        canvas.lines(x,y,color='black')
        canvas.point(xs,ys,color='black',size=200)
        canvas.point(xe,ye,color='black',size=200)
        canvas.update()
    return he,le

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
    #canvas = cnv.Canvas((-1.2*rmax,1.2*rmax,-1.2*rmax,1.2*rmax))
    #robot = rb.body(xs, ys, hs, canvas)
    planner = db.Planner(rb.turning_radius, 0.2)
    
    hhe = []
    hhe1 = []
    hhe2 = []
    hhe3 = []
    hhe4 = []
    # Path
    for r in np.linspace(rmax, rmin, 4):
        for a in np.linspace(0,2*pi,36):
            xe = r*np.cos(a)
            ye = r*np.sin(a)
            he,le = play(planner,xs,ys,hs,xe,ye,canvas=None)
            he1 = db.heading(xs,ys,xe,ye)
            he2 = 0.5*(hs + he1)
            he3 = db.sample(planner.planner,xs,ys,hs,xe,ye)
            he4 = planner.optimal(xs,ys,hs,xe,ye)
            print(f'Angle {np.rad2deg(a)}: optmal length {le}, heading: {he} {he1} {he2} {he3} {he4}')
            hhe.append(he)
            hhe1.append(he1)
            hhe2.append(he2)
            hhe3.append(he3)
            hhe4.append(he4)
    hhe = np.array(hhe)
    hhe1 = np.array(hhe1)
    hhe2 = np.array(hhe2)
    hhe3 = np.array(hhe3)
    hhe4 = np.array(hhe4)
    #canvas.show()
    plt.figure()
    plt.plot(hhe,label='he')
    #plt.plot(hhe1,label='he1')
    #plt.plot(hhe2,label='he2')
    plt.plot(hhe3,label='he3')
    plt.plot(hhe4,label='he4')
    plt.legend()
    plt.grid()
    plt.show()
    
if __name__ == "__main__":
    main()