import numpy as np
import DEM as dm
import Geotiff as gt
import Visualization as vz

datase = gt.open('data/mesh/models/part1')

x,y,Z = gt.dem(datase)

print(Z.shape)
print(dm.shape(x,y))

vz.plotMesh(x,y,Z)

X,Y,Z = dm.flatten(x,y,Z)
print(X.shape)
print(Y.shape)
print(Z.shape)

Z = dm.unflatten(x,y,Z)
print(Z.shape)

vz.plotMesh(x,y,Z)


