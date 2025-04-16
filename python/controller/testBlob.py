import numpy as np
import matplotlib.pyplot as plt
from random import random 
import Canvas as cv
import Particle as pt
import Geotiff as gt
import DEM as dm
import Visualization as vz

from scipy.ndimage import label
from skimage.morphology import disk, binary_erosion, binary_dilation, binary_opening, binary_closing
from scipy.ndimage import binary_fill_holes
from skimage import filters
from skimage import measure
from scipy.spatial import ConvexHull, convex_hull_plot_2d

def imin(x):
    i = np.argmin(x)
    if i.size > 1:
        i = i[0]
    return i

def rand(rmin=0.0, rmax=1.0):
    return (rmax-rmin)*random() + rmin

def fillNan(v, a = 0):
    x = v.copy()
    x[np.isnan(x)] = a
    return x

def norm(v):
    vmin = np.nanmin(v)
    vmax = np.nanmax(v)
    return (v - vmin)/(vmax - vmin)

def image():
    path = 'data/mesh/terrains/mars'
    database = gt.open(path + '-dsc')
    x,y,z = gt.dem(database)
    return z

def mask(img, tresh = 0.05):
    m = img.copy()
    m = norm(m)
    m[m<tresh] = 0
    m[m!=0] = 1
    return m.astype(int)

def border(image, n=1):
    img = image.copy()
    img[ 0:n,:] = 0
    img[-n: ,:] = 0
    img[:, 0:n] = 0
    img[:,-n:] = 0
    return img

def main():

    img = image()
    img = fillNan(img)
    img = filters.gaussian(img, sigma=4)
    m = mask(img)
    m = binary_erosion(m)
    m = border(m)
    m = binary_fill_holes(m)
    contours = measure.find_contours(m)
    
    hulls = []
    for contour in contours:
        hull = ConvexHull(contour, True)
        vertices = contour[hull.vertices]
        hulls.append(vertices)

    fig, ax = plt.subplots()
    ax.imshow(m, cmap=plt.cm.gray)

    for i in range( len(contours) ):
        c = contours[i]
        h = hulls[i]
        ax.plot(c[:, 1], c[:, 0], linewidth=10)
        ax.plot(h[:, 1], h[:, 0], linewidth=10)
        plt.pause(2)
  
    plt.show(block=True)

if __name__ == "__main__":
    main() 
