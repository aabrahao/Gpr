import robot.geometry.geotiff as gt
import robot.geometry.dem as dm
import robot.geometry.pointcloud as pc
import robot.geometry.stl as stl

import robot.visualizer.plotter.surface as vz

vz.plotStl('data/mesh/mars')

mesh = stl.load('data/mesh/mars')
x,y,z = stl.vertices(mesh)
vz.info(x,y,z,'Pointcloud')
vz.plotPoints(x,y,z,z)

dx = 1 # m
xs,ys,zs = pc.resample(x,y,z,dx, method='linear')
vz.info(xs,ys,zs,'Resampled')
vz.compare(z,zs)
vz.plotMesh(xs,ys,zs,zs,edges=True)

