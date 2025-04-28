import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import robot.visualizer.plotter.canvas as cnv
import robot.visualizer.plotter.shapes as shp

def main():
    canvas = cnv.Canvas((-100,100,-100,100))

    r1 = shp.Rectangle(0,0,5,3, color='green', canvas=canvas,fill=False)
    r2 = shp.Rectangle(0,0,10,20,fill=False, canvas=canvas,thickness=4)
    c1 = shp.Circle(0,0,6,canvas=canvas,fill=False)

    canvas.add(c1)

    b1 = shp.Block([r1,r2,c1],-40,-40,np.pi/3, color = 'red')

    canvas.add(b1)

    t = 0
    tmax = 100
    dt = 0.1
    while (t <= tmax):
        x = t*np.cos(t)
        y = t*np.sin(t) 
        angle = np.pi*t
        r1.move(x,y)
        r2.rotate(angle)
        c1.move(2*x,2*y)
        
        xb1,yb1 = b1.position()
        b1.move(xb1,yb1+y)
        b1.rotate(angle)

        canvas.update()
        t += dt
    canvas.show()
    
if __name__ == "__main__":
    main()