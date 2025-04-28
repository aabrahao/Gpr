import robot.analysis.hydrology as hyd
import robot.geometry.geotiff as gt
import robot.geometry.dem as dm
import robot.visualizer.plotter.surface as vz
import numpy as np
import robot.geometry.stl as st
from scipy import stats as sts
import matplotlib.pylab as plt

import robot.numerics.array as nm
import robot.analysis.quantification as qty
import robot.analysis.mcdm as mcdm

import robot.geometry.clustering.Clustering as cls

def main():

    mcdm.setProject('data/mcdm/mars', save= False)
    
    x,y,Z = mcdm.open()

    criteria = {
        'pit'    : [1.0, 0.005, mcdm.Data.MASK],
        'twi'    : [0.6,   1.4, mcdm.Data.RAW],
        'radmean': [0.6,  0.01, mcdm.Data.PREDICTION],
        'radstd' : [0.4,  0.01, mcdm.Data.UNCERTAINTY],
        'hst'    : [0.7,   0.0, mcdm.Data.RAW]
    }
      
    maps = mcdm.load(criteria)
    scores = mcdm.rank(criteria, maps)
    score = mcdm.judge(criteria, scores)
    
    level = .2
    mask = nm.mask(score, level)
    #mcdm.plot(x,y,Z,mask)

    # Antenna coverage
    lx,ly = dm.lot(x,y)
    antenna = 0.01*lx 
    overlap = 0.2

    # Run the algorithm
    xp, yp = dm.points(x,y,mask)
    
    #cls.benchmark(xp,yp,antenna,overlap)
    # GPR antenna cover    
    xc,yc,xr,yr = cls.cluster(xp, yp, antenna, overlap)
    cls.visualize(xp,yp,xc,yc,xr,yr,antenna,overlap)
    e = nm.vector()
    cls.visualize(xc,yc,xc,yc,e,e,antenna,overlap)
    
if __name__ == "__main__":
    main()
