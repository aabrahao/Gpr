# Claude AI

import numpy as np
from scipy import spatial
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from time import time
import heapq

def two_stage_greedy(x, y, radius, overlap):
    """
    Two-stage greedy algorithm for circle coverage:
    1. Fast initial placement without overlap constraints
    2. Constraint optimization on remaining points
    
    Args:
        x, y: Point coordinates
        radius: Circle radius
        overlap: Maximum allowed overlap percentage between circles
    
    Returns:
        xc, yc: Circle centers with overlap constraints satisfied
        xr, yr: Circle centers with relaxed overlap constraints
    """
    start_time = time()
    points = np.column_stack((x, y))
    
    # Stage 1: Fast initial placement (no constraints)
    print("Stage 1: Fast initial placement...")
    initial_centers = fast_initial_placement(points, radius)
    
    # Mark covered points based on initial circles
    covered = mark_covered_points(points, initial_centers, radius)
    
    # Stage 2: Constraint optimization on remaining points
    print("Stage 2: Constraint optimization...")
    xc, yc, xr, yr = constraint_optimization(
        points, initial_centers, covered, radius, overlap
    )
    
    # Performance statistics
    end_time = time()
    stats(x, y, xc, yc, xr, yr, end_time - start_time)
    
    return xc, yc, xr, yr

def fast_initial_placement(points, radius):
    """
    Places circles quickly without considering overlap constraints.
    Uses a grid-based approach for large datasets.
    
    Args:
        points: Nx2 array of point coordinates
        radius: Circle radius
    
    Returns:
        centers: Array of initial circle centers
    """
    # Create spatial index for efficient neighborhood queries
    tree = spatial.KDTree(points)
    
    # Use priority queue for selecting best centers
    # Each entry is (-score, point_idx) so we get highest scores first
    candidates = []
    
    # Grid approach for very large datasets
    if len(points) > 10000:
        # Divide space into grid cells roughly 2*radius in size
        x_min, y_min = np.min(points, axis=0)
        x_max, y_max = np.max(points, axis=0)
        
        cell_size = 2 * radius
        grid_x = np.arange(x_min, x_max + cell_size, cell_size)
        grid_y = np.arange(y_min, y_max + cell_size, cell_size)
        
        # For each grid cell, find point closest to center with most neighbors
        for i in range(len(grid_x) - 1):
            for j in range(len(grid_y) - 1):
                cell_center = np.array([
                    (grid_x[i] + grid_x[i+1]) / 2,
                    (grid_y[j] + grid_y[j+1]) / 2
                ])
                
                # Find points in this cell
                mask_x = (points[:, 0] >= grid_x[i]) & (points[:, 0] < grid_x[i+1])
                mask_y = (points[:, 1] >= grid_y[j]) & (points[:, 1] < grid_y[j+1])
                cell_points_idx = np.where(mask_x & mask_y)[0]
                
                if len(cell_points_idx) > 0:
                    # Find point in cell with most neighbors within radius
                    cell_points = points[cell_points_idx]
                    
                    # For each point in cell, count neighbors within radius
                    for idx in cell_points_idx:
                        neighbors = tree.query_ball_point(points[idx], r=radius)
                        score = len(neighbors)
                        heapq.heappush(candidates, (-score, idx))
    else:
        # For smaller datasets, evaluate all points
        for i in range(len(points)):
            neighbors = tree.query_ball_point(points[i], r=radius)
            score = len(neighbors)
            heapq.heappush(candidates, (-score, i))
    
    # Greedy selection of centers
    centers = []
    covered = np.zeros(len(points), dtype=bool)
    
    while candidates and not np.all(covered):
        # Get best candidate
        _, idx = heapq.heappop(candidates)
        
        # Skip if already covered
        if covered[idx]:
            continue
        
        # Add as center
        centers.append(points[idx])
        
        # Mark its neighbors as covered
        neighbors = tree.query_ball_point(points[idx], r=radius)
        covered[neighbors] = True
    
    return np.array(centers)

def mark_covered_points(points, centers, radius):
    """
    Mark points covered by initial centers.
    
    Args:
        points: Nx2 array of point coordinates
        centers: Array of circle centers
        radius: Circle radius
    
    Returns:
        covered: Boolean array indicating which points are covered
    """
    covered = np.zeros(len(points), dtype=bool)
    tree = spatial.KDTree(points)
    
    for center in centers:
        neighbors = tree.query_ball_point(center, r=radius)
        covered[neighbors] = True
    
    return covered

def constraint_optimization(points, initial_centers, covered, radius, overlap):
    """
    Refine circles to satisfy overlap constraints.
    
    Args:
        points: Nx2 array of point coordinates
        initial_centers: Array of initial circle centers
        covered: Boolean array indicating which points are covered
        radius: Circle radius
        overlap: Maximum allowed overlap percentage
    
    Returns:
        xc, yc: Circle centers with overlap constraints satisfied
        xr, yr: Circle centers with relaxed overlap constraints
    """
    tree = spatial.KDTree(points)
    max_overlap_distance = 2 * radius * (1 - overlap)
    
    # Start with initial centers
    circles = []
    relaxed = []
    
    # Check which initial centers satisfy overlap constraints
    for center in initial_centers:
        center_array = np.array([center])
        
        # Check distance to existing circles
        valid = True
        if circles:
            circles_array = np.array(circles)
            distances = np.linalg.norm(center_array - circles_array, axis=1)
            if np.any(distances < max_overlap_distance):
                valid = False
        
        if valid:
            circles.append(center)
        else:
            relaxed.append(center)
    
    # Handle remaining uncovered points
    uncovered = ~covered
    uncovered_indices = np.where(uncovered)[0]
    
    if len(uncovered_indices) > 0:
        # Use original greedy algorithm for remaining points
        uncovered_points = points[uncovered_indices]
        
        while len(uncovered_indices) > 0:
            # Vectorized radius query
            neighbors_list = [
                tree.query_ball_point(point, r=radius) 
                for point in uncovered_points
            ]
            
            # Count uncovered neighbors
            coverage_counts = np.array([
                np.sum(~covered[indices]) for indices in neighbors_list
            ])
            
            # Calculate overlap distances
            all_existing = np.array(circles + relaxed)
            if len(all_existing) > 0:
                dist_to_existing = np.linalg.norm(
                    uncovered_points[:, None, :] - all_existing[None, :, :],
                    axis=2
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
            else:
                # Relax constraints if no valid candidates
                best_idx = np.argmax(coverage_counts)
                best_center = uncovered_points[best_idx]
                relaxed.append(best_center)
            
            # Update covered points
            best_neighbors = neighbors_list[best_idx]
            covered[best_neighbors] = True
            
            # Update uncovered indices and points
            uncovered = ~covered
            uncovered_indices = np.where(uncovered)[0]
            uncovered_points = points[uncovered_indices]
            
            if len(uncovered_indices) == 0:
                break
    
    # Extract coordinates
    circles = np.array(circles)
    relaxed = np.array(relaxed)
    
    if len(circles) > 0:
        xc, yc = circles[:, 0], circles[:, 1]
    else:
        xc, yc = np.array([]), np.array([])
    
    if len(relaxed) > 0:
        xr, yr = relaxed[:, 0], relaxed[:, 1]
    else:
        xr, yr = np.array([]), np.array([])
    
    return xc, yc, xr, yr

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
        circle = Circle((xi, yi), radius, fill=True, color='red', alpha=0.2)
        ax.add_patch(circle)
    plt.scatter(xr, yr, c='red', s=30, alpha=0.8, label='Relaxed circles')
    
    # Plot constrained circles
    for xi, yi in zip(xc, yc):
        circle = Circle((xi, yi), radius, fill=True, color='green', alpha=0.2)
        ax.add_patch(circle)
    plt.scatter(xc, yc, c='green', s=30, alpha=0.8, label='Constrained circles')
    
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()

# Example usage
if __name__ == "__main__":
    # Generate random points
    np.random.seed(42)
    n_points = 5000  # Try with 5000 points first, then scale up
    x = np.random.rand(n_points) * 1000
    y = np.random.rand(n_points) * 1000
    
    # Parameters
    radius = 50
    overlap = 0.1  # 10% overlap allowed
    
    # Run two-stage algorithm
    xc, yc, xr, yr = two_stage_greedy(x, y, radius, overlap)
    
    # Visualize
    visualize(x, y, xc, yc, xr, yr, radius, overlap)