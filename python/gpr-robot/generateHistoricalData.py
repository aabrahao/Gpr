import numpy as np
import robot.geometry.geotiff as gt
import robot.geometry.dem as dm
import robot.visualizer.plotter.surface as vz
import numpy as np
import robot.numerics.array as nm

def main():
    vz.saveImages(False)

    path = 'data/mesh/terrains/mars'

    dataset = gt.open(path)
    x,y,Z = gt.dem(dataset)
    vz.plotMesh(x,y,Z,Z,view=(45,5), title='DEM')

    H = nm.trim(Z, 7.0, 13.0)
    H = nm.normalize(H)
    H = nm.nanify(H, 0.0)
    dm.save(x,y,H,path+'hst')
    vz.plotMesh(x,y,Z,H,view=(45,5), title='hst')

    # Test
    dataset = gt.open(path+'hst')
    vz.plotDem(dataset,view=(45,5), title='Ok?')

if __name__ == "__main__":
    main()
