import numpy as np
from sklearn.neighbors import KDTree
from scipy.spatial import cKDTree as KDTree
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

import Numeric as nm

def greedy1(x, y , radius, overlap):
    overlap = 100*overlap
    points = nm.points(x,y)
    # Create a KD-tree from the points
    tree = KDTree(points)
    # Initialize variables
    covered = np.zeros(len(points), dtype=bool)  # Boolean array instead of set
    circles = []  # Circles that respect overlap constraint
    relaxed = []   # Circles that had to relax constraint
    max_overlap_distance = 2 * radius * (1 - overlap/100)
    # For all points...
    while not np.all(covered):
        best_center = None
        best_coverage_count = 0
        best_covered_mask = None
        # Only consider uncovered points as potential centers
        uncovered_indices = np.where(~covered)[0]
        for i in uncovered_indices:
            # Find all points within radius of this center
            indices = tree.query_radius([points[i]], r=radius)[0]
            # Calculate how many new points this circle would cover
            new_covered_mask = ~covered[indices]
            new_coverage_count = np.sum(new_covered_mask)
            # Check if this circle satisfies overlap constraints with existing circles
            valid_placement = True
            for existing_center in circles + relaxed:
                dist = np.linalg.norm(points[i] - existing_center)
                if dist < max_overlap_distance:
                    valid_placement = False
                    break
            # Update best center if this is better and valid
            if valid_placement and new_coverage_count > best_coverage_count:
                best_center = points[i]
                best_coverage_count = new_coverage_count
                best_covered_mask = indices
        # If we found a valid circle placement
        if best_center is not None:
            circles.append(best_center)
            covered[best_covered_mask] = True
        else:
            # No valid placement with overlap constraint - relax constraint for remaining
            best_fallback_center = None
            best_fallback_coverage = 0
            best_fallback_covered = None
            for i in uncovered_indices:
                indices = tree.query_radius([points[i]], r=radius)[0]
                new_coverage_count = np.sum(~covered[indices])
                if new_coverage_count > best_fallback_coverage:
                    best_fallback_center = points[i]
                    best_fallback_coverage = new_coverage_count
                    best_fallback_covered = indices
            # Add this circle even if it violates overlap constraint
            if best_fallback_center is not None:
                relaxed.append(best_fallback_center)
                covered[best_fallback_covered] = True
    xc, yc = nm.xy(np.array(circles))
    xr, yr = nm.xy(np.array(relaxed))
    stats(x, y, xc,yc,xr,yr)

    return xc, yc, xr, yr

def stats(x, y, xc, yc, xr, yr):
    n = len(x)
    nc = xc.size
    nr = xr.size
    print('-------------------------------------')
    print('Greedy clustering stats')
    print(f"Total points: {n}")
    print(f"Covered (respecting overlap): {nc}")
    print(f"Relaxed (overlap constraint relaxed): {nr}")
    print(f"Total circles: {nc + nr}")
    print('-------------------------------------')

def greedy2(x, y, radius, overlap):
    overlap = 100*overlap
    points = nm.points(x, y)
    # Create a KD-tree from the points
    tree = KDTree(points)
    # Initialize variables
    covered = np.zeros(len(points), dtype=bool)
    circles = []
    relaxed = []
    max_overlap_distance = 2 * radius * (1 - overlap/100)
    # For all points
    while not np.all(covered):
        best_center = None
        best_coverage_count = 0
        best_covered_mask = None
        # Get uncovered points
        uncovered_indices = np.where(~covered)[0]
        if len(uncovered_indices) == 0:
            break
        # Find all points within radius of each uncovered point
        # This is a vectorized approach that calculates all potential centers at once
        uncovered_points = points[uncovered_indices]
        # For each potential center, find how many uncovered points it would cover
        coverage_counts = np.zeros(len(uncovered_indices), dtype=int)
        covered_indices_list = []
        # Query the KD tree for each potential center
        # This could be done in batches for large datasets
        for i, idx in enumerate(uncovered_indices):
            indices = tree.query_radius([points[idx]], r=radius)[0]
            new_covered = ~covered[indices]
            coverage_counts[i] = np.sum(new_covered)
            covered_indices_list.append(indices)
        # Check overlap constraints for all potential centers
        valid_placements = np.ones(len(uncovered_indices), dtype=bool)
        if len(circles) + len(relaxed) > 0:
            all_circles = np.array(circles + relaxed)
            # Calculate all distances in a vectorized way
            for i, idx in enumerate(uncovered_indices):
                if valid_placements[i]:  # Only check if still valid
                    # Calculate distances to all existing circles
                    distances = np.linalg.norm(all_circles - points[idx], axis=1)
                    # Check if any distance is less than max_overlap_distance
                    if np.any(distances < max_overlap_distance):
                        valid_placements[i] = False
        # Find the best valid placement
        if np.any(valid_placements):
            # Only consider valid placements
            valid_coverage = coverage_counts * valid_placements
            best_idx = np.argmax(valid_coverage)
            if valid_coverage[best_idx] > 0:
                best_center_idx = uncovered_indices[best_idx]
                best_center = points[best_center_idx]
                best_covered_mask = covered_indices_list[best_idx]
                circles.append(best_center)
                covered[best_covered_mask] = True
                continue
        # If no valid placement found, relax constraints
        best_fallback_idx = np.argmax(coverage_counts)
        if coverage_counts[best_fallback_idx] > 0:
            best_fallback_center_idx = uncovered_indices[best_fallback_idx]
            best_fallback_center = points[best_fallback_center_idx]
            best_fallback_covered = covered_indices_list[best_fallback_idx]
            relaxed.append(best_fallback_center)
            covered[best_fallback_covered] = True
    # Convert circle centers back to separate x and y coordinates
    circles = np.array(circles)
    relaxed = np.array(relaxed)
    xc, yc = nm.xy(circles)
    xr, yr = nm.xy(relaxed)
    # Analyse performance
    stats(x, y, xc, yc, xr, yr)
    return xc, yc, xr, yr

def greedy(x, y, radius, overlap):
    points = np.column_stack((x, y))
    tree = KDTree(points)

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

    # Performance statistics (assuming `stats` function exists)
    stats(x, y, xc, yc, xr, yr)

    return xc, yc, xr, yr

def visualize(x,y,xc,yc,xr,yr,radius):
    beginCanvas(x,y)
    drawPoints(x, y, 'black')
    drawCircles(xc, yc, radius, 'green')
    drawCircles(xr, yr, radius, 'red')
    endCanvas(x,y)

def drawPoints(x,y,color):
    plt.scatter(x, y, c='black', s=10, label='Points')
    plt.pause(1)

def drawCircles(x,y,r,color):
    ax = plt.gca() 
    for xc, yc in zip(x,y):
        circle = Circle((xc,yc), r, fill=True, color=color, alpha=0.25)
        ax.add_patch(circle)
        plt.scatter(xc, yc, c=color, s=50, alpha=0.5)
    plt.pause(1)

def beginCanvas(x,y):
    plt.figure()
    plt.title(f'Greedy cover algorithm with constant radius algorithm')
    plt.legend(['Points', 'Covered (overlap ≤ {overlap}%)', 'Relaxed (unconstrained)'])
    plt.axis('equal')
    plt.xlim(np.min(x),np.max(x))
    plt.ylim(np.min(y),np.max(y))
    plt.grid(True)

def endCanvas(x,y):
    plt.show()
