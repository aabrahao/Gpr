import numpy as np
from scipy.spatial import KDTree
import matplotlib.pyplot as plt

import ClusteringNoOverlap as ct

def visualize(x, y, xc, yc, xr, yr, r):
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.scatter(x,y, s=2, alpha=0.4, label='Points')

    for cx, cy in zip(xc,yc):
        circle = plt.Circle((cx,cy), r, color='green', fill=False, linewidth=1.5)
        ax.add_artist(circle)
    
    for rx, ry in zip(xr,yr):
        circle = plt.Circle((rx,ry), r, color='red', fill=False, linewidth=1.5)
        ax.add_artist(circle)

    ax.scatter(xc,yc, color='red', s=10, label='Circle Centers')
    ax.set_aspect('equal', adjustable='box')
    plt.legend()
    plt.title('Strictly Non-overlapping Circles')
    plt.show()

if __name__ == "__main__":
    np.random.seed(42)
    n = 1000
    x = np.random.rand(n)
    y = np.random.rand(n)
    radius = 0.1

    xc, yc, xr, yr = ct.no_overlap(x, y, radius)
    print(f'No overlap: {len(xc)}')
    print(f'Overlap: {len(xr)}')
    print(f'Total: {len(xr)+len(xc)}')
    
    visualize(x, y, xc, yc, xr, yr, radius)


    
