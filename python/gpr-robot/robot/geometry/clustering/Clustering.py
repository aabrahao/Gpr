import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from time import time

import robot.numerics.array as nm

import ClusteringOverlapConstraintGreedy as cgc
import ClusteringOverlapConstraintTwoStage1 as ct1
import ClusteringOverlapConstraintTwoStage2 as ct2
import ClusteringMinimumCircleCover as ct3
import ClusteringFastCover as ct4
import ClusteringNoOverlap as ct5

def benchmark(x,y,r,o):
    #cluster(x,y,r,o,greedy)
    cluster(x,y,r,o,twoStage1)
    cluster(x,y,r,o,twoStage2)
    #cluster(x,y,r,o,minimalCover)
    cluster(x,y,r,o,fastCover)
    cluster(x,y,r,o,noOverlap)

def cluster(x, y, radius, overlap, algorithm=None):
    if algorithm is None:
        algorithm = noOverlap
    start = time()
    xc, yc, xr, yr = algorithm(x, y, radius, overlap)
    end = time()
    stats(x, y, xc, yc, xr, yr, end-start)
    return xc, yc, xr, yr

def noOverlap(x, y, radius, overlap):
    xc, yc, xr, yr = ct5.no_overlap(x, y, radius)
    return  xc, yc, xr, yr

def greedy(x, y, radius, overlap):
    return cgc.greedy(x, y, radius, overlap)

def twoStage1(x, y, radius, overlap):
    return ct1.two_stage_greedy(x, y, radius, overlap)

def twoStage2(x, y, radius, overlap):
    return ct2.two_stage_covering_algorithm(x, y, radius, overlap)

def minimalCover(x, y, radius, overlap):
    xc, yc = ct3.minimal_circles_cover(x, y, radius)
    e = nm.vector()
    return xc, yc, e, e

def fastCover(x, y, radius, overlap):
    xc, yc = ct4.cover_points_with_circles(x, y, radius)
    e = nm.vector()
    return xc, yc, e, e

def stats(x, y, xc, yc, xr, yr, elapsed_time):
    """Display performance statistics for the algorithm."""
    n = len(x)
    nc = len(xc)
    nr = len(xr)
    print('-------------------------------------')
    print('Two-stage greedy clustering stats')
    print(f"Total points: {n}")
    print(f"Covered (respecting overlap): {nc}")
    print(f"Relaxed (overlap constraint relaxed): {nr}")
    print(f"Total circles: {nc + nr}")
    print(f"Elapsed time: {elapsed_time:.2f} seconds")
    print('-------------------------------------')

def visualize(x, y, xc, yc, xr, yr, radius, overlap):
    """Visualize the results."""
    plt.figure(figsize=(12, 10))
    plt.title(f'Two-stage greedy cover algorithm (radius={radius}, overlap={overlap*100}%)')
    
    # Plot points
    plt.scatter(x, y, c='black', s=5, label='Points')
    
    # Plot circles
    ax = plt.gca()
    
    # Plot relaxed circles first (so they're underneath)
    for xi, yi in zip(xr, yr):
        circle = Circle((xi, yi), radius, fill=True, color='red', alpha=0.5)
        ax.add_patch(circle)
    plt.scatter(xr, yr, c='red', s=30, alpha=0.8, label='Relaxed circles')
    
    # Plot constrained circles
    for xi, yi in zip(xc, yc):
        circle = Circle((xi, yi), radius, fill=True, color='blue', alpha=0.5)
        ax.add_patch(circle)
    plt.scatter(xc, yc, c='green', s=30, alpha=0.8, label='Constrained circles')
    
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()