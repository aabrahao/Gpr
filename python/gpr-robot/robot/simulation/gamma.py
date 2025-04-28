import numpy as np
import robot.numerics.array as nm
import robot.geometry.dem as dm

class Point:
    def __init__(self, x, y, strength, radious):
        self.x = x
        self.y = y
        self.s = strength
        self.r = radious
    
    def equation(self, r):
        return self.r/(4*np.pi*r*r)

    def points(self,x,y):
        r = nm.distance(x,y,self.x,self.y)
        r[r<=self.r] = self.r
        i = self.equation(r)
        return i
    
    def dem(self,x,y):
        xp, yp = dm.points(x,y)
        zp = self.points(xp, yp)
        return dm.unflatten(x,y,zp)
