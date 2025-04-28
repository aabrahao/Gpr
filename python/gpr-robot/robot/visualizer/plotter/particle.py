import numpy as np
import matplotlib.pyplot as plt

class Particle:
    def __init__(self, canvas, x, y, color = None, trace = True, alpha = 0.5):
        self.trace = trace
        if trace:
            self.x = [x]
            self.y = [y]
            self.line = canvas.axes.plot(x,y,alpha=alpha,color=color)[0]
            self.particle = canvas.axes.plot(x,y,'o',alpha=alpha,color=color)[0]
        else:
            self.x = x
            self.y = y
            self.particle = canvas.axes.plot(x,y,'o',color=color)[0]
    def move(self, x, y):
        if self.trace:
            self.x.append(x)
            self.y.append(y)
            self.line.set_data(self.x, self.y)
            self.particle.set_data(x, y)
        else:
            self.x = x
            self.y = y
            self.particle.set_data(x, y)
    def inside(self,x,y,r):
        if abs(self.x - x) <= r and abs(self.y - y) <= r:
            print('Inside!')
            return True
        else:
            return False
    def clear(self):
        if self.trace:
            self.x = []
            self.y = []
            self.line.set_data([],[])
            self.particle.set_data([],[])
        else:
            self.x = 0
            self.y = 0
            self.particle.set_data([],[])
    def position(self):
        return self.x[-1], self.y[-1]
