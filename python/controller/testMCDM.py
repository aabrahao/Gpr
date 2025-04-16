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

    # uncertanties = qt.qualify(maps[3])
    # priorities = qt.extract(priority, np.linspace(0,1,10))
    # plot(x,y,Z,priorities)
    #for z in np.linspace(0,1,10):
    #    P = nm.trim(priority, z)
    #    plot(x,y,Z,P)
    #    print(z)


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
    rankings = mcdm.rank(criteria, maps)
    priority = mcdm.judge(criteria, rankings)
    
    levels = np.linspace(0,1,10)
    priorities = mcdm.emphasize(priority, levels)
    mcdm.plot(x,y,Z,priorities)

if __name__ == "__main__":
    main()
