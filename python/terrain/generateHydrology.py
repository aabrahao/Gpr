
import os
import glob
import Hydrology as hyd
import Geotiff as gt

#hyd.setProject('data/taudem/enogeo', 'raw')
#hyd.generate()

#hyd.setProject('data/taudem/logan', 'raw')
#hyd.generate()

#hyd.setProject('data/rifle/terrain', 'masked')
#hyd.generate()

#hyd.setProject('data/rifle/terrain', 'raw')
#hyd.generate()

hyd.setProject('data/mesh/terrains', 'mars')
hyd.generate()

#hyd.setProject('data/mesh', 'part1')
#hyd.generate()
