import numpy as np
import robot.geometry.stl as stl
import robot.visualizer.plotter.surface as vz

mesh = stl.load('data/mesh/mars')
x,y,z = stl.vertices(mesh)
vz.plotPoints(x,y,z,z)
