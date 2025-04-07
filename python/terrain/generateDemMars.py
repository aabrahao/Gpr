import Geotiff as gt
import Dem as dm
import Pointcloud as pc
import Stl as stl

import Visualization as vz

path = 'data/mesh/mars'

mesh = stl.load(path)
x,y,z = stl.vertices(mesh)

# Gazeabo
# <scale>0.1 0.1 0.025</scale>
x = 0.1*x
y = 0.1*y
z = 0.1*z
dx = 0.1*1 # m

vz.info(x,y,z,'Pointcloud')
vz.plotPoints(x,y,z,z)

xs,ys,zs = pc.resample(x,y,z,dx, method='nearest')
xs,ys,zs = dm.shrink(xs,ys,zs,100)

vz.info(xs,ys,zs,'Resampled')
vz.compare(z,zs)
vz.plotMesh(xs,ys,zs,zs,edges=True)

dm.save(xs,ys,zs,path)

# Checking
database = gt.open(path)
gt.info(database)
vz.plotDem(database)
