import numpy as np
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import Matern

import Numeric as nm
import DEM as dm

class Estimator:
    def __init__(self, length_scale=1.0, nu=1.5):
        kernel = Matern(length_scale=length_scale, nu=nu)
        self.gp = GaussianProcessRegressor(kernel=kernel)
    def fit(self,x,y,z):
        points =  nm.points(x,y)
        self.gp.fit(points,z)
    def points(self,x,y):
        points = nm.points(x,y)
        m, s = self.gp.predict(points, return_std=True)
        return m, s
    def dem(self,x,y):
        xp, yp = dm.points(x,y)
        zp, sp = self.points(xp, yp)
        zp = dm.unflatten(x,y,zp)
        sp = dm.unflatten(x,y,sp)
        return zp,sp
