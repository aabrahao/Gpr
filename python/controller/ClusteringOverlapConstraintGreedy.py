import numpy as np
from scipy import spatial
from sklearn.cluster import DBSCAN
#from scipy.spatial import cKDTree as KDTree
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from time import time
from joblib import Parallel, delayed

import Numeric as nm

def greedy(x, y, radius, overlap):
    points = np.column_stack((x, y))
    tree = spatial.KDTree(points)

    covered = np.zeros(len(points), dtype=bool)
    circles, relaxed = [], []
    max_overlap_distance = 2 * radius * (1 - overlap)

    while not np.all(covered):
        uncovered_indices = np.flatnonzero(~covered)
        uncovered_points = points[uncovered_indices]

        # Vectorized radius query
        neighbors_list = tree.query_ball_point(uncovered_points, r=radius)
        coverage_counts = np.array([np.sum(~covered[indices]) for indices in neighbors_list])

        # Calculate overlap distances in vectorized form
        all_existing = np.array(circles + relaxed)
        if all_existing.size > 0:
            dist_to_existing = np.linalg.norm(
                uncovered_points[:, None, :] - all_existing[None, :, :], axis=2
            )
            valid_mask = np.all(dist_to_existing >= max_overlap_distance, axis=1)
        else:
            valid_mask = np.ones(len(uncovered_points), dtype=bool)

        # Select best valid candidate
        valid_coverage_counts = coverage_counts * valid_mask
        if np.any(valid_coverage_counts):
            best_idx = np.argmax(valid_coverage_counts)
            best_center = uncovered_points[best_idx]
            circles.append(best_center)
            covered[neighbors_list[best_idx]] = True
        else:
            # Relax constraints if no valid candidates
            best_idx = np.argmax(coverage_counts)
            best_center = uncovered_points[best_idx]
            relaxed.append(best_center)
            covered[neighbors_list[best_idx]] = True

    # Extract coordinates
    circles = np.array(circles)
    relaxed = np.array(relaxed)

    xc, yc = circles[:, 0], circles[:, 1]
    xr, yr = relaxed[:, 0], relaxed[:, 1]

    return xc, yc, xr, yr
