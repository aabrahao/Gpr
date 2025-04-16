import numpy as np
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

import Clustering as cls
import Numeric as nm

def main():

    # Coverage
    radius = 1.5
    overlap = 0.2

    # Gid
    n = 1001
    width = 20
    hieght = 20
    
    x = width*nm.random(n)
    y = hieght*nm.random(n)

    # Algorithm
    xc,yc,xr,yr = cls.greedy(x, y, radius, overlap)

    # Results
    cls.visualize(x,y,xc,yc,xr,yr,radius)

if __name__ == "__main__":
    main()