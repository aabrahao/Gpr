import Hydrology as hyd
import Geotiff as gt
import Dem as dm
import Visualization as vz
import numpy as np
import Stl as st

import matplotlib.pylab as plt

def main():
    #hyd.setProject('data/taudem/enogeo', 'raw')
    #hyd.setProject('data/taudem/logan', 'raw')
    #hyd.setProject('data/rifle/terrain', 'masked')
    #hyd.setProject('data/rifle/terrain', 'raw')

    hyd.setProject('data/mesh/terrains', 'mars')
    view=(45,5)

    # Mesh
    database = gt.open( hyd.project() )
    x,y,z = gt.dem(database)
    vz.plotMesh(x,y,z,z,hyd.project(),view=view)

    # Hydology
    files = hyd.outputs()
    for file in files:
        output = gt.open(file)
        gt.info(output)
        c = gt.data(output)
        vz.plotMesh(x,y,z,c,file,view=view)

if __name__ == "__main__":
    main()
