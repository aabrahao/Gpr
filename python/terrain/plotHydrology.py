import Hydrology as hyd
import Geotiff as gt
import Dem as dm
import Visualization as vz
import numpy as np

import matplotlib.pylab as plt

#hyd.setProject('data/taudem/enogeo', 'raw')
#hyd.setProject('data/taudem/logan', 'raw')
hyd.setProject('data/rifle/terrain', 'masked')
#hyd.setProject('data/rifle/terrain', 'raw')
#hyd.setProject('data/mesh', 'mars')

database = gt.open( hyd.project() )
x,y,z = gt.dem(database)

files = hyd.outputs()
for file in files:
    output = gt.open(file)
    gt.info(output)
    c = gt.data(output)
    vz.plotMesh(x,y,z,c,file)