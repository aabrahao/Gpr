import Geotiff as gt
import Dem as dm
import Pointcloud as pc
import Stl as stl

import Visualization as vz

path = 'data/mesh/part1'

vz.plotStl(path)

mesh = stl.load(path)
x,y,z = stl.vertices(mesh)

dx = 0.01 

vz.info(x,y,z,'Pointcloud')
vz.plotPoints(x,y,z,z)

xs,ys,zs = pc.resample(x,y,z,dx, method='linear')
#xs,ys,zs = dm.shrink(xs,ys,zs,50)

vz.info(xs,ys,zs,'Resampled')
vz.compare(z,zs)
vz.plotMesh(xs,ys,zs,zs,edges=True)

dm.save(xs,ys,zs,path)

# Checking
database = gt.open(path)
gt.info(database)
vz.plotDem(database)
