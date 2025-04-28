
import os
import glob
import robot.analysis.hydrology as hyd
import robot.geometry.geotiff as gt
import robot.visualizer.plotter.surface as vz
import numpy as np

import matplotlib.pylab as plt

def spaced_cells(size, cell, start):
    cells = (size-start)//cell
    end = size - cells*cell - start
    return start, cells, end

def check_spaced(size, cell, start, cells, end):
    n = start + cells*cell + end
    print(f'{size} = {start} + {cells}*{cell} + {end} = {n}')

#hyd.setProject('data/rifle/terrain', 'masked')
hyd.setProject('data/rifle/terrain', 'raw')

database = gt.open( hyd.project() )
gt.info(database)
x,y,z = gt.dem(database)

nx, ny = z.shape

sx, cx, ex = spaced_cells(nx, 19, 17) 
sy, cy, ey = spaced_cells(ny, 19, 12)

check_spaced(nx, 19, sx, cx, ex)
check_spaced(ny, 19, sy, cy, ey)

plt.subplot(211)
plt.plot(x[0,:],z[0,:],'-o')
plt.title(f'Row ({len(z[0,:])})')

plt.subplot(212)
plt.plot(y[:,0],z[:,0],'-o')
plt.title(f'Column ({len(z[:0])})')
plt.show()