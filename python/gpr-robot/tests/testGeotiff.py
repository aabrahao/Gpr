
import robot.geometry.geotiff as gt
import robot.visualizer.plotter.surface as vz
import robot.geometry.dem as dm

database = gt.open('data/mesh/mars')
gt.info(database)
vz.plotDem(database)

x,y,z = gt.dem(database)
vz.plotMesh(x,y,z)

