import numpy as np
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from time import time
from joblib import Parallel, delayed

def fast_circle_coverage(x, y, radius, overlap_percent=20, bin_factor=10):
    """
    Fast circle coverage algorithm for large point sets using spatial binning
    
    Parameters:
    -----------
    x, y : array-like
        Coordinates of points
    radius : float
        Radius of each circle
    overlap_percent : float
        Maximum allowed overlap between circles as percentage
    bin_factor : int
        Controls bin size (smaller = more bins, potentially faster but more memory)
    
    Returns:
    --------
    xc, yc : arrays
        Coordinates of standard circles (respecting overlap constraint)
    xr, yr : arrays
        Coordinates of relaxed circles (exceeding overlap constraint)
    """
    start_time = time()
    points = np.column_stack((x, y))
    n_points = len(points)
    
    print(f"Processing {n_points} points...")
    
    # Calculate bin size based on radius and bin_factor
    bin_size = radius / bin_factor
    
    # Determine grid dimensions
    x_min, y_min = np.min(points, axis=0)
    x_max, y_max = np.max(points, axis=0)
    
    # Add padding
    x_min -= radius
    y_min -= radius
    x_max += radius
    y_max += radius
    
    # Calculate number of bins
    n_bins_x = int(np.ceil((x_max - x_min) / bin_size))
    n_bins_y = int(np.ceil((y_max - y_min) / bin_size))
    
    print(f"Creating grid with {n_bins_x} x {n_bins_y} bins...")
    
    # Assign points to bins
    bin_indices_x = np.minimum(
        np.floor((points[:, 0] - x_min) / bin_size).astype(int),
        n_bins_x - 1
    )
    bin_indices_y = np.minimum(
        np.floor((points[:, 1] - y_min) / bin_size).astype(int),
        n_bins_y - 1
    )
    
    # Create dictionary of points in each bin
    grid = {}
    for i in range(n_points):
        bin_key = (bin_indices_x[i], bin_indices_y[i])
        if bin_key not in grid:
            grid[bin_key] = []
        grid[bin_key].append(i)
    
    # Sort bins by number of points (descending)
    sorted_bins = sorted(
        [(bin_key, len(points_in_bin)) for bin_key, points_in_bin in grid.items()],
        key=lambda x: x[1],
        reverse=True
    )
    
    # Parameters for circle placement
    covered = np.zeros(n_points, dtype=bool)
    standard_circles = []
    relaxed_circles = []
    max_overlap_distance = 2 * radius * (1 - overlap_percent/100)
    
    # Process bins in order of decreasing point density
    for bin_key, _ in sorted_bins:
        # Skip if all points in this bin are already covered
        if all(covered[i] for i in grid[bin_key]):
            continue
        
        # Find all points within the neighborhood of this bin
        x_bin, y_bin = bin_key
        neighborhood_points = []
        
        # Define neighborhood radius in bins
        bin_radius = int(np.ceil(radius / bin_size)) + 1
        
        # Collect points from neighborhood bins
        for dx in range(-bin_radius, bin_radius + 1):
            for dy in range(-bin_radius, bin_radius + 1):
                neighbor_key = (x_bin + dx, y_bin + dy)
                if neighbor_key in grid:
                    neighborhood_points.extend(grid[neighbor_key])
        
        if not neighborhood_points:
            continue
        
        # Create a local KD-tree for this neighborhood
        local_points = points[neighborhood_points]
        local_covered = covered[neighborhood_points]
        local_tree = KDTree(local_points)
        
        # Consider each uncovered point in the current bin as a potential center
        for point_idx in grid[bin_key]:
            if covered[point_idx]:
                continue
                
            # Find all points within radius of this center
            query_point = points[point_idx].reshape(1, -1)
            indices = local_tree.query_radius(query_point, r=radius)[0]
            
            if len(indices) == 0:
                continue
                
            # Map back to original indices
            original_indices = [neighborhood_points[i] for i in indices]
            
            # Calculate how many new points this circle would cover
            new_covered_count = np.sum(~covered[original_indices])
            
            if new_covered_count == 0:
                continue
                
            # Check overlap constraints with existing circles
            valid_placement = True
            
            if standard_circles or relaxed_circles:
                all_circles = np.array(standard_circles + relaxed_circles)
                distances = np.linalg.norm(all_circles - points[point_idx], axis=1)
                if np.any(distances < max_overlap_distance):
                    valid_placement = False
            
            # Place the circle
            if valid_placement:
                standard_circles.append(points[point_idx])
                covered[original_indices] = True
            else:
                # Check if this would be the best relaxed circle
                if new_covered_count > 0:
                    relaxed_circles.append(points[point_idx])
                    covered[original_indices] = True
    
    # Convert to arrays
    standard_circles = np.array(standard_circles)
    relaxed_circles = np.array(relaxed_circles)
    
    end_time = time()
    print(f"Processing completed in {end_time - start_time:.2f} seconds")
    print(f"Points covered: {np.sum(covered)} of {n_points}")
    print(f"Standard circles: {len(standard_circles)}")
    print(f"Relaxed circles: {len(relaxed_circles)}")
    print(f"Total circles: {len(standard_circles) + len(relaxed_circles)}")
    
    # Extract x and y coordinates
    if len(standard_circles) > 0:
        xc, yc = standard_circles[:, 0], standard_circles[:, 1]
    else:
        xc, yc = np.array([]), np.array([])
    
    if len(relaxed_circles) > 0:
        xr, yr = relaxed_circles[:, 0], relaxed_circles[:, 1]
    else:
        xr, yr = np.array([]), np.array([])
    
    return xc, yc, xr, yr

# Function to plot the results (with sampling for large datasets)
def plot_results(x, y, xc, yc, xr, yr, radius, max_points=10000):
    n_points = len(x)
    
    # Sample points if there are too many
    if n_points > max_points:
        print(f"Sampling {max_points} points for visualization...")
        indices = np.random.choice(n_points, max_points, replace=False)
        x_sample = x[indices]
        y_sample = y[indices]
    else:
        x_sample = x
        y_sample = y
    
    plt.figure(figsize=(12, 12))
    
    # Draw standard circles in green
    for i in range(len(xc)):
        circle_fill = Circle((xc[i], yc[i]), radius, color='green', alpha=0.15)
        plt.gca().add_patch(circle_fill)
        circle_border = Circle((xc[i], yc[i]), radius, fill=False, color='green', 
                              alpha=0.7, linewidth=2.5)
        plt.gca().add_patch(circle_border)
    
    # Draw relaxed circles in red
    for i in range(len(xr)):
        circle_fill = Circle((xr[i], yr[i]), radius, color='red', alpha=0.15)
        plt.gca().add_patch(circle_fill)
        circle_border = Circle((xr[i], yr[i]), radius, fill=False, color='red', 
                              alpha=0.7, linewidth=2.5, linestyle='--')
        plt.gca().add_patch(circle_border)
    
    # Plot points on top
    plt.scatter(x_sample, y_sample, c='blue', s=10, alpha=0.8, zorder=10)
    
    # Plot circle centers
    if len(xc) > 0:
        plt.scatter(xc, yc, c='green', s=30, alpha=1.0, marker='o', edgecolors='black')
    if len(xr) > 0:
        plt.scatter(xr, yr, c='red', s=30, alpha=1.0, marker='o', edgecolors='black')
    
    # Set equal aspect ratio
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True, alpha=0.3)
    plt.title(f'Circle Coverage: {len(xc)} standard + {len(xr)} relaxed circles')
    plt.tight_layout()
    plt.show()

# Example usage for large dataset
if __name__ == "__main__":
    # Generate 50,000 random points
    np.random.seed(42)
    n_points = 50000
    x = np.random.uniform(0, 1000, n_points)
    y = np.random.uniform(0, 1000, n_points)
    
    # Run the algorithm
    radius = 20
    overlap_percent = 20
    xc, yc, xr, yr = fast_circle_coverage(x, y, radius, overlap_percent)
    
    # Plot results
    plot_results(x, y, xc, yc, xr, yr, radius)