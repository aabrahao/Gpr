
import Geotiff as gt
import Visualization as vz
import Dem as dm

database = gt.open('data/mesh/mars')
gt.info(database)
vz.plotDem(database)

x,y,z = gt.dem(database)
vz.plotMesh(x,y,z)

