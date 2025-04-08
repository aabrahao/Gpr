import System as sys
import Geotiff as gt
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import Print as prt
import Cad as cad

def main():
    data = gt.Dataset('data/rifle/drone.tif')
    
    ax = cad.plots(1,1)
    data.show(ax); cad.title(ax, data.name)
    cad.pause()
    
if __name__ == "__main__":
    main() 
