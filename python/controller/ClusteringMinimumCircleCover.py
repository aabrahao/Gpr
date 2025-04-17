# Claude.Ai
import numpy as np
from scipy import spatial
from sklearn.cluster import DBSCAN, KMeans
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from time import time

def minimal_circles_cover(x, y, radius):
    """
    Finds a minimal set of circles with given radius to cover all points.
    
    Args:
        x, y: Point coordinates
        radius: Circle radius
    
    Returns:
        centers_x, centers_y: Coordinates of circle centers
    """
    start_time = time()
    points = np.column_stack((x, y))
    
    # Estimate the minimum number of circles needed using packing density
    # This is a theoretical lower bound based on area coverage
    x_min, y_min = np.min(points, axis=0)
    x_max, y_max = np.max(points, axis=0)
    area = (x_max - x_min) * (y_max - y_min)
    circle_area = np.pi * radius**2
    estimated_circles = int(1.1 * area / circle_area)  # Add 10% margin
    
    print(f"Estimated minimum circles needed: {estimated_circles}")
    
    # Use K-means clustering to find potential circle centers
    n_clusters = min(estimated_circles, len(points))
    kmeans = KMeans(n_clusters=n_clusters, random_state=42, n_init=10)
    kmeans.fit(points)
    
    # Refine the centers using a greedy approach
    centers = refine_centers(points, kmeans.cluster_centers_, radius)
    
    # Performance statistics
    end_time = time()
    print('-------------------------------------')
    print('Minimal circles cover stats:')
    print(f"Total points: {len(points)}")
    print(f"Total circles: {len(centers)}")
    print(f"Elapsed time: {end_time - start_time:.2f} seconds")
    print('-------------------------------------')
    
    centers = np.array(centers)
    return centers[:, 0], centers[:, 1]

def refine_centers(points, initial_centers, radius):
    """
    Refines the initial centers using a greedy approach to
    minimize the number of circles needed.
    
    Args:
        points: Nx2 array of point coordinates
        initial_centers: Initial circle centers from K-means
        radius: Circle radius
    
    Returns:
        centers: Refined list of circle centers
    """
    tree = spatial.KDTree(points)
    covered = np.zeros(len(points), dtype=bool)
    centers = []
    
    # Add centers that cover the most uncovered points until all points are covered
    while not np.all(covered):
        # Find the best circle center that covers the most uncovered points
        best_coverage = 0
        best_center = None
        
        # Try centers near clusters and uncovered points
        candidate_centers = []
        
        # Add initial centers from K-means as candidates
        for center in initial_centers:
            if not any(np.linalg.norm(center - c) < 1e-10 for c in centers):
                candidate_centers.append(center)
        
        # Add uncovered points as potential candidates
        uncovered_indices = np.where(~covered)[0]
        if len(uncovered_indices) > 0:
            # Don't add all uncovered points as candidates, just select a subset
            # based on the density of their neighborhood
            if len(uncovered_indices) > 1000:
                # For large datasets, sample points or use a different strategy
                sample_size = min(1000, len(uncovered_indices))
                sampled_indices = np.random.choice(uncovered_indices, size=sample_size, replace=False)
                candidate_centers.extend(points[sampled_indices])
            else:
                candidate_centers.extend(points[uncovered_indices])
        
        # Evaluate all candidate centers
        for center in candidate_centers:
            neighbors = tree.query_ball_point(center, r=radius)
            coverage = np.sum(~covered[neighbors])
            
            if coverage > best_coverage:
                best_coverage = coverage
                best_center = center
        
        # If we found a good center, add it
        if best_center is not None and best_coverage > 0:
            centers.append(best_center)
            neighbors = tree.query_ball_point(best_center, r=radius)
            covered[neighbors] = True
        else:
            # If no center covers any uncovered points, add a center at an uncovered point
            uncovered_indices = np.where(~covered)[0]
            if len(uncovered_indices) > 0:
                center = points[uncovered_indices[0]]
                centers.append(center)
                neighbors = tree.query_ball_point(center, r=radius)
                covered[neighbors] = True
            else:
                # All points are covered
                break
    
    return centers

def post_optimization(points, centers, radius):
    """
    Post-optimization to potentially reduce the number of circles.
    
    Args:
        points: Nx2 array of point coordinates
        centers: List of circle centers
        radius: Circle radius
    
    Returns:
        optimized_centers: Optimized list of circle centers
    """
    # Convert to numpy arrays for vectorized operations
    points = np.array(points)
    centers = np.array(centers)
    
    # Create a coverage matrix: centers x points
    coverage = np.zeros((len(centers), len(points)), dtype=bool)
    
    # For each center, determine which points it covers
    for i, center in enumerate(centers):
        distances = np.linalg.norm(points - center, axis=1)
        coverage[i] = distances <= radius
    
    # Try to eliminate redundant centers
    redundant = np.zeros(len(centers), dtype=bool)
    
    for i in range(len(centers)):
        # Skip if already marked as redundant
        if redundant[i]:
            continue
        
        # Points covered by this center
        covered_by_i = coverage[i]
        
        # Check if all these points are covered by other non-redundant centers
        covered_elsewhere = np.zeros(len(points), dtype=bool)
        for j in range(len(centers)):
            if i != j and not redundant[j]:
                covered_elsewhere |= coverage[j]
        
        # If all points covered by i are also covered elsewhere,
        # then center i is redundant
        if np.all(covered_by_i <= covered_elsewhere):
            redundant[i] = True
    
    # Return non-redundant centers
    optimized_centers = centers[~redundant]
    return optimized_centers