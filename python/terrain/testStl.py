import numpy as np
import Stl as stl
import Visualization as vz

mesh = stl.load('data/mesh/mars')
x,y,z = stl.vertices(mesh)
vz.plotPoints(x,y,z,z)
