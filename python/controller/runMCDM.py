import Hydrology as hyd
import Geotiff as gt
import DEM as dm
import Visualization as vz
import numpy as np
import Stl as st
from scipy import stats as sts
import matplotlib.pylab as plt

import Numeric as nm
import Quantification as qty
import MCDM as mcdm

import Clustering as cls

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
    antenna = 0.1*lx 
    overlap = 0.2

    # Run the algorithm
    xp, xp = 
    xc,yc,xr,yr = cls.greedy2(x, y, antenna, overlap)
    cls.visualize(xc,yc,xr,yr,antenna)

if __name__ == "__main__":
    main()
