import Hydrology as hyd
import Geotiff as gt
import DEM as dm
import Visualization as vz
import numpy as np
import Stl as st

import matplotlib.pylab as plt

def plot(path, name,view):
    # Dataset
    hyd.setProject(path, name)
    # Mesh
    dataset = gt.open( hyd.project() )
    x,y,z = gt.dem(dataset)
    vz.plotMesh(x,y,z,z,title=hyd.project(),view=view)
    vz.plotMesh(x,y,z,title=hyd.project()+'grid',view=view, edges=True)
    # Hydology
    files = hyd.outputs()
    for file in files:
        output = gt.open(file)
        gt.info(output)
        c = gt.data(output)
        vz.plotMesh(x,y,z,c,file,view=view)

def main():
    #vz.saveImages(True)
    vz.saveImages(False)
    #plot('data/mesh/terrains','mars',view=(45.0,5.0))
    #plot('data/mesh/models','part1', view=(-45.0,5))
    plot('data/mcdm','mars', view=(45.0,5.0))

if __name__ == "__main__":
    main()