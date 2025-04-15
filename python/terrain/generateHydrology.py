
import os
import glob
import Hydrology as hyd
import Geotiff as gt

hyd.setProject('data/mesh/terrains', 'mars')
hyd.generate()

#hyd.setProject('data/mesh/models', 'part1')
#hyd.generate()
