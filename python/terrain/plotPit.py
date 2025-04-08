import Hydrology as hyd
import Geotiff as gt
import Dem as dm
import Visualization as vz
import numpy as np

import matplotlib.pylab as plt

#hyd.setProject('data/taudem/enogeo', 'raw')
#hyd.setProject('data/taudem/logan', 'raw')
#hyd.setProject('data/rifle/terrain', 'raw')
#hyd.setProject('data/rifle/terrain', 'masked')
#hyd.setProject('data/mesh', 'mars')
hyd.setProject('data/mesh', 'part1')

terrain = gt.open( hyd.project() )
x,y,z = gt.dem(terrain)

filled = gt.open( hyd.project() + 'fel' )
xf,yf,zf = gt.dem(filled)

# Pits
zp = zf - z

vz.plotMesh(x,y,z,zp)