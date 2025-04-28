import numpy as np
from time import sleep
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import robot.utilities.numerics as nm

def limits(x,y):
    xmin = np.nanmin(x)
    xmax = np.nanmax(x)
    ymin = np.nanmin(y)
    ymax = np.nanmax(y)
    return [xmin, xmax, ymin, ymax] 

class Canvas: # limis: [xmin, xmax, ymin, ymax]
    def __init__(self, limits=(-1.0, 1.0, -1.0, 1.0), size=(18,18), grid=None):
        self.create(limits,size, grid)
    def create(self, limits, size, grid):
        figure = plt.figure(figsize=size)
        self.axes = figure.add_subplot()
        if grid:
            self.axes.grid(True)
        self.axes.figure.tight_layout()
        self.axes.set_aspect('equal')
        self.axes.axis(limits)
        self.axes.figure.show()
    def zoom(self, limits):
        self.axes.axis(limits)
        self.axes.figure.tight_layout()
    def show(self, block=True):
        self.axes.figure.canvas.draw()
        if block:
            self.axes.figure.canvas.start_event_loop(timeout=-1) 
        else:
           self.axes.figure.canvas.flush_events() 
    def update(self):
        self.axes.figure.canvas.draw()
        self.axes.figure.canvas.flush_events()
    def line(self,x1,y1,x2,y2,color=None,alpha=0.5,thickness=None):
        self.axes.plot((x1,x2),(y1, y2),
                       color=color,
                       linewidth=thickness,
                       alpha=alpha)
    def label(self,x,y,label):
        if not label:
            return
        plt.annotate(label,(x,y),xytext=(5,5),textcoords='offset points')
    def point(self,x,y,color=None,size=None,alpha=0.5,label=''):
        self.axes.plot(x,y,'o',
                          markersize=size,
                          markerfacecolor=color,
                          markeredgecolor=color,
                          alpha=alpha)
        self.label(x,y,label)
    def lines(self,x,y,color=None,pause=None,thickness=None,alpha=0.5):
        if pause is None:
            self.axes.plot(x,y,color=color,
                           linewidth=thickness,alpha=alpha)
        else:
            for i in range(len(x)-1):
                self.line(x[i],y[i],x[i+1],y[i+1],
                          color=color,width=thickness,alpha=alpha)
                self.update()
                sleep(pause)
    def labels(self,x,y,labels='indices'):
        if labels == 'indices':
            for i in range(len(x)):
                self.label(x[i],y[i],label=str(i))
    def points(self,x,y,color=None,size=None,labels='',pause=None,alpha=0.5):
        if pause is None:
            self.axes.plot(x,y,'o',
                          markersize=size,
                          markerfacecolor=color,
                          markeredgecolor=color,
                          alpha=alpha)
            self.labels(x,y,labels)
        else:
            for i in range(len(x)):
                if labels == 'indices':
                    label = str(i)
                else:
                    label = ''
                self.point(x[i],y[i],color=color,
                           size=size,label=label,alpha=alpha)
                self.update()
                sleep(pause)
    def add(self, shape):
        shape._attach(self)      

    def circle(self,x,y,r,color=None,thickness=None,fill=True,alpha=0.5):
        circle = patches.Circle((x,y),radius=r,color=color,
                                linewidth=thickness, fill=fill, alpha=alpha)
        self.axes.add_patch(circle)
    def circles(self,x,y,r,color=None,thickness=None, fill=True,alpha=0.5):
        if nm.scalar(r):
            rr = 0*x + r
        else:
            rr = r
        for xp,yp,rp in zip(x,y,rr):
            self.circle(xp,yp,rp,color=color,
                        thickness=thickness,fill=fill,alpha=alpha)
    