import Geotiff as gt
import DEM as dm
import Visualization as vz

#database = gt.open('data/taudem/logan/raw')
#database = gt.open('data/rifle/terrain/raw')
database = gt.open('data/rifle/terrain/masked')

x,y,z = gt.dem(database)
vz.info(x,y,z,'Dem')
vz.plotMesh(x,y,z,z)

dx, dy = gt.resolution(database)

xs,ys,zs = dm.resample(x,y,z,15, method='linear')
vz.info(xs,ys,zs,'Resampled')
vz.plotMesh(xs,ys,zs,zs,edges=True)

xs,ys,zs = dm.resample(xs,ys,zs,0.5, method='linear')
vz.info(xs,ys,zs,'Resampled')
vz.plotMesh(xs,ys,zs,zs,edges=True)