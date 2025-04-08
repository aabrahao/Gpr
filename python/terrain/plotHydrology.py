import Hydrology as hyd
import Geotiff as gt
import Dem as dm
import Visualization as vz
import numpy as np

import matplotlib.pylab as plt

#hyd.setProject('data/taudem/enogeo', 'raw')
#hyd.setProject('data/taudem/logan', 'raw')
#hyd.setProject('data/rifle/terrain', 'masked')
#hyd.setProject('data/rifle/terrain', 'raw')

hyd.setProject('data/mesh', 'mars')
view=(45,5)

#hyd.setProject('data/mesh', 'part1')
#view=(-45.0,5)

vz.plotStl(hyd.project(),view=view)

# Project
database = gt.open( hyd.project() )
x,y,z = gt.dem(database)
vz.plotMesh(x,y,z,z,hyd.project(),view=view)

# Pit
filled = gt.open( hyd.project() + 'fel' )
xf,yf,zf = gt.dem(filled)
zp = zf - z
vz.plotMesh(x,y,z,zp,hyd.project()+'pit',view=view)

# Hydology
files = hyd.outputs()
for file in files:
    output = gt.open(file)
    gt.info(output)
    c = gt.data(output)
    vz.plotMesh(x,y,z,c,file,view=view)
