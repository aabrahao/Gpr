
import os
import glob
import robot.analysis.hydrology as hyd
import robot.geometry.geotiff as gt
import robot.visualizer.plotter.surface as vz
import numpy as np

import matplotlib.pylab as plt


def reduce(matrix, cell_size=20):
    flat = matrix.flatten()
    n_cells = len(flat) // cell_size
    reduced = []
    for i in range(n_cells):
        start_idx = i * cell_size
        center_idx = start_idx + cell_size // 2
        reduced.append(flat[center_idx])
    return np.array(reduced)


def space_cells(size, cell):
    cells = size//cell
    remainder = size%c
    start = remainder//2
    end = remainder-start
    return start, cells, end


#hyd.setProject('data/taudem/enogeo', 'raw')
#hyd.setProject('data/taudem/logan', 'raw')
#hyd.setProject('data/rifle/terrain', 'masked')
hyd.setProject('data/rifle/terrain', 'raw')

database = gt.open( hyd.project() )
gt.info(database)
x,y,z = gt.dem(database)
#vz.plot(x,y,z,z)

ny, nx = z.shape

c = 19
sx, cx, ex = space_cells(nx, c) 
sy, cy, ey = space_cells(ny, c)

print(nx)
print(ny)
print(sx + cx*c + ex)
print(sy + cy*c + ey)

r = z[0]
print(r.shape)

plt.plot(r,'-o')
plt.show()

c = z[:, 0]
print(c.shape)

plt.plot(c,'-o')
plt.show()