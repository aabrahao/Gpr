import numpy as np
from scipy.spatial import KDTree
import matplotlib.pyplot as plt

def no_overlap(x,y,radius):
    xp = x.copy()
    yp = y.copy()
    xc,yc,indices = find_non_overlapping_circles(xp, yp, radius)
    xr = np.array([],dtype=float)
    yr = np.array([],dtype=float)
    while True:
        xp,yp,n = remove(xp,yp, indices)
        print(f'Uncovered: {n}')
        if n == 0:
            break
        # Relaxed with overlaped
        xrc, yrc, indices = find_non_overlapping_circles(xp, yp, radius)
        xr = np.append(xr,xrc)
        yr = np.append(yr,yrc)
    return xc, yc, xr, yr

def remove(x, y, indices):
    i = flatten(indices)
    xr = np.delete(x, i)
    yr = np.delete(y, i)
    return xr, yr, len(xr)

def flatten(indices):
    all = [ ]
    for inners in indices:
        all.extend(inners)
    all = np.array(all)
    all = np.unique(all) # Just in case!
    return all

def find_non_overlapping_circles(x, y, r):
    points = np.column_stack((x, y))
    points = np.asarray(points)
    N = len(points)

    point_tree = KDTree(points)

    densities = np.array([len(point_tree.query_ball_point(p, r)) for p in points])
    sorted_indices = np.argsort(-densities)

    circle_centers = []
    circles_point_indices = []
    centers_tree = None

    covered = np.zeros(N, dtype=bool)

    for idx in sorted_indices:
        if covered[idx]:
            continue

        candidate_center = points[idx]

        if centers_tree is not None:
            if centers_tree.query_ball_point(candidate_center, 2 * r):
                continue

        circle_centers.append(candidate_center)
        indices_in_circle = point_tree.query_ball_point(candidate_center, r)
        circles_point_indices.append(indices_in_circle)
        covered[indices_in_circle] = True

        centers_tree = KDTree(circle_centers)

    circle_centers = np.array(circle_centers)
    xc,yc = circle_centers[:, 0], circle_centers[:, 1]
    return xc, yc, circles_point_indices
