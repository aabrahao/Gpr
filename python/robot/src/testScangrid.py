import numpy as np
import matplotlib.pyplot as plt

import System as sys
import Geotiff as gt
import Print as prt
import Cad as cad
import Geometry as gmt
import Path as pth
import Vector as vc

from math import ceil

def toMeter(f):
    return 0.3048*f

def main():

    png = 'grid-30-10-45.png'

    # Grid
    offset = -toMeter(60) # m
    spacing = toMeter(60) # m
    angle = 0

    print(offset)
    
    # Satellite image
    cell = gt.Dataset('data/rifle/satellite/raw.tif')

    # Elevation
    z = gt.Dataset('data/rifle/terrain/masked.tif')
    xb,yb = z.boundary()
    xf, yf =  gmt.offset(xb, yb, offset)

    # Draw
    ax1 = cad.plots()
    cad.dataset(ax1, cell)
    cad.lines(ax1, xb, yb, 'black', thickness=3)
    #cad.lines(ax1, xf, yf, 'blue')
    
    x, y = pth.generateScangrid(xf, yf, spacing, vc.toRadian(angle))
    cad.lines(ax1, x, y, color='black', thickness=1)
    cad.points(ax1, x, y, color='black')
    cad.pause()

    #plt.savefig(png, bbox_inches='tight')

    print('Good job!')
    
if __name__ == "__main__":
    main() 
