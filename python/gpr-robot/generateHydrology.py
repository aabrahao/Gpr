
import os
import glob
import robot.analysis.hydrology as hyd
import robot.geometry.geotiff as gt

hyd.setProject('data/mesh/terrains', 'mars')
hyd.generate()

#hyd.setProject('data/mesh/models', 'part1')
#hyd.generate()
