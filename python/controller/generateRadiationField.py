import numpy as np
import matplotlib.pylab as plt

import Hydrology as hyd
import Geotiff as gt
import DEM as dm
import Visualization as vz
import numpy as np
import Stl as st
import Numeric as nm

import GaussianProcess as gp
import GammaSource as gs

def generateRandomField(x,y,Z, sources, max_strength, radious):
    xs, ys, zs = dm.sample(x,y,Z, sources)
    ss = nm.rand( sources )
    I = 0*Z
    for i in range(sources):
        s = gs.Point(xs[i], ys[i], ss[i], radious)
        I = I + s.dem(x,y)
        print(f'Gamma source: {i} of {sources}')
    return I

def generate(path, view, sources, strength, radious, training):
    # Dataset
    dataset = gt.open( path )
    gt.info(dataset)
    x,y,Z = gt.dem(dataset)
    # Field
    I = generateRandomField(x,y,Z,sources,strength,radious)
    vz.plotMesh(x,y,Z,I,path+'rad',view=view)
    # Training
    xt,yt,it = dm.sample(x,y,I,training)
    estimator = gp.Estimator()
    estimator.fit(xt,yt,it)
    # Prediction
    IM, IS = estimator.dem(x,y)
    vz.plotMesh(x,y,Z,IM,path + 'radmean',view=view)
    vz.plotMesh(x,y,Z,IS,path + 'radstd',view=view)
    # Save
    dm.save(x,y,I ,path+'rad')
    dm.save(x,y,IM,path+'radmean')
    dm.save(x,y,IS,path+'radstd')

def main():
    vz.saveImages(True)
    #generate('data/mesh/terrains/mars',
    #          view=(45,5), 
    #          sources = 100, 
    #          strength = 1.0, 
    #          radious  = 5.0, 
    #          training = 100)
    #generate('data/mesh/models/part1', 
    #         view=(-45.0,5), 
    #         sources = 100,
    #         strength = 1.0, 
    #         radious = 0.2, 
    #         training = 20)
    
    print('All done!')    

if __name__ == "__main__":
    main()
