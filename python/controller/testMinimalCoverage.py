"""
Optimized Circle Coverage Algorithm for 50,000+ Points
Using NumPy, SciPy, and Scikit-learn for high performance

This implementation guarantees complete coverage of all points.
"""

import numpy as np
from scipy.spatial import cKDTree
from sklearn.cluster import KMeans
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import math

class CircleCoverage:
    def __init__(self, points, radius):
        """
        Initialize the circle coverage solver
        
        Args:
            points: numpy array of shape (n, 2) containing the points to cover
            radius: radius of each circle
        """
        self.points = np.asarray(points)
        self.radius = radius
        self.n_points = len(points)
        
        # Build KD-Tree for efficient spatial queries
        self.kdtree = cKDTree(self.points)
        
        # Track covered points
        self.covered = np.zeros(self.n_points, dtype=bool)
        self.circle_centers = []
        
    def generate_candidate_centers(self, n_candidates=None):
        """
        Generate candidate circle centers using multiple strategies
        
        Args:
            n_candidates: approximate number of candidates to generate
            
        Returns:
            numpy array of candidate centers
        """
        # Use uncovered points as candidates - GUARANTEED to cover all points
        uncovered_points = self.points[~self.covered]
        
        # If we have too many uncovered points, use K-means to reduce
        if n_candidates and len(uncovered_points) > n_candidates:
            kmeans = KMeans(
                n_clusters=min(n_candidates, len(uncovered_points)), 
                init='k-means++',
                n_init=1,  # Faster initialization
                max_iter=20  # Limit iterations for speed
            )
            kmeans.fit(uncovered_points)
            return kmeans.cluster_centers_
        
        return uncovered_points
    
    def generate_grid_candidates(self, density=0.7):
        """
        Generate a grid of candidate circle centers
        
        Args:
            density: density factor (lower means more spread out grid)
            
        Returns:
            numpy array of candidate centers
        """
        # Get bounds of points
        min_x, min_y = np.min(self.points, axis=0)
        max_x, max_y = np.max(self.points, axis=0)
        
        # Add margin
        min_x -= self.radius
        min_y -= self.radius
        max_x += self.radius
        max_y += self.radius
        
        # Optimal grid spacing is based on radius
        grid_spacing = self.radius * 2 * density
        
        # Create grid
        x_grid = np.arange(min_x, max_x, grid_spacing)
        y_grid = np.arange(min_y, max_y, grid_spacing)
        xx, yy = np.meshgrid(x_grid, y_grid)
        
        # Convert to array of points
        grid_points = np.column_stack((xx.ravel(), yy.ravel()))
        
        return grid_points
    
    def find_points_within_radius(self, center):
        """
        Find all points within radius of center using KD-Tree
        
        Args:
            center: (x, y) coordinates of center
            
        Returns:
            indices of points within radius
        """
        return self.kdtree.query_ball_point(center, self.radius)
    
    def evaluate_centers_vectorized(self, centers, batch_size=1000):
        """
        Evaluate multiple centers in a vectorized way for efficiency
        
        Args:
            centers: array of center coordinates to evaluate
            batch_size: batch size for processing to manage memory usage
            
        Returns:
            array of coverage counts for each center
        """
        n_centers = len(centers)
        coverage_counts = np.zeros(n_centers, dtype=int)
        
        # Process in batches to manage memory
        for i in range(0, n_centers, batch_size):
            batch_centers = centers[i:i+batch_size]
            batch_counts = np.zeros(len(batch_centers), dtype=int)
            
            # For each center in batch
            for j, center in enumerate(batch_centers):
                indices = self.find_points_within_radius(center)
                if indices:
                    batch_counts[j] = np.sum(~self.covered[indices])
            
            coverage_counts[i:i+len(batch_centers)] = batch_counts
            
        return coverage_counts
    
    def add_circle(self, center):
        """
        Add a circle at the given center and mark points as covered
        
        Args:
            center: (x, y) coordinates of center
        """
        self.circle_centers.append(center)
        indices = self.find_points_within_radius(center)
        self.covered[indices] = True
    
    def solve_greedy(self, use_grid=True, adaptive=True):
        """
        Solve the circle coverage problem using a greedy approach with guarantee
        
        Args:
            use_grid: whether to use grid-based candidates initially
            adaptive: whether to adapt candidate generation strategy
            
        Returns:
            list of circle centers
        """
        start_time = time.time()
        
        # Initialize progress tracking
        total_covered = 0
        remaining = self.n_points
        
        # Main greedy loop
        iteration = 0
        print(f"Starting coverage of {self.n_points} points with radius {self.radius}...")
        
        while total_covered < self.n_points:
            iteration += 1
            
            # PHASE 1: Try grid-based approach for efficiency in early iterations
            if use_grid and (iteration == 1 or (not adaptive and total_covered < 0.9 * self.n_points)):
                candidates = self.generate_grid_candidates()
                coverage_counts = self.evaluate_centers_vectorized(candidates)
                
                # Find best candidate
                if np.max(coverage_counts) > 0:
                    best_idx = np.argmax(coverage_counts)
                    best_center = candidates[best_idx]
                    self.add_circle(best_center)
                    
                    # Update progress
                    prev_covered = total_covered
                    total_covered = np.sum(self.covered)
                    newly_covered = total_covered - prev_covered
                    remaining = self.n_points - total_covered
                    
                    # Continue to next iteration if we made progress
                    if newly_covered > 0:
                        if iteration % 50 == 0 or total_covered == self.n_points:
                            elapsed = time.time() - start_time
                            print(f"Iteration {iteration}: {total_covered}/{self.n_points} covered "
                                f"({total_covered/self.n_points:.1%}) with {len(self.circle_centers)} circles. "
                                f"Elapsed: {elapsed:.1f}s")
                        continue
            
            # PHASE 2: Switch to using uncovered points directly as candidates
            # This guarantees we will cover all remaining points
            remaining_points = self.points[~self.covered]
            
            if len(remaining_points) == 0:
                break
                
            # OPTIMIZATION: If few points remain, place circles directly on them
            if len(remaining_points) <= 100 or total_covered > 0.95 * self.n_points:
                # For the last few points, put a circle on each uncovered point
                # This ensures we cover everything even if it's inefficient
                for point in remaining_points:
                    self.add_circle(point)
                    
                total_covered = self.n_points  # All points are now covered
                remaining = 0
                break
            
            # For more points, use K-means to find strategic centers
            n_candidates = min(100, max(10, len(remaining_points) // 10))
            kmeans = KMeans(
                n_clusters=n_candidates, 
                init='k-means++',
                n_init=1,
                max_iter=20
            )
            kmeans.fit(remaining_points)
            candidates = kmeans.cluster_centers_
            
            # Evaluate candidates
            coverage_counts = self.evaluate_centers_vectorized(candidates)
            
            # Find best candidate
            best_idx = np.argmax(coverage_counts)
            best_center = candidates[best_idx]
            self.add_circle(best_center)
            
            # Update progress
            prev_covered = total_covered
            total_covered = np.sum(self.covered)
            newly_covered = total_covered - prev_covered
            remaining = self.n_points - total_covered
            
            # Report progress
            if iteration % 50 == 0 or total_covered == self.n_points:
                elapsed = time.time() - start_time
                print(f"Iteration {iteration}: {total_covered}/{self.n_points} covered "
                      f"({total_covered/self.n_points:.1%}) with {len(self.circle_centers)} circles. "
                      f"Elapsed: {elapsed:.1f}s")
            
            # Emergency measure to guarantee progress
            if newly_covered == 0:
                # Place a circle directly on an uncovered point
                center = remaining_points[0]
                self.add_circle(center)
                total_covered = np.sum(self.covered)
        
        # Final statistics
        elapsed = time.time() - start_time
        print(f"\nFinal solution: {len(self.circle_centers)} circles for {total_covered}/{self.n_points} points "
              f"({total_covered/self.n_points:.1%})")
        print(f"Average coverage: {total_covered/len(self.circle_centers):.1f} points per circle")
        print(f"Completed in {elapsed:.2f} seconds")
        
        return np.array(self.circle_centers)
    
    def visualize_solution(self, max_points=5000, figsize=(12, 10)):
        """
        Visualize the solution
        
        Args:
            max_points: maximum number of points to plot
            figsize: figure size
        """
        plt.figure(figsize=figsize)
        
        # Plot a sample of points if there are too many
        if len(self.points) > max_points:
            indices = np.random.choice(len(self.points), max_points, replace=False)
            sample_points = self.points[indices]
            plt.scatter(sample_points[:, 0], sample_points[:, 1], s=2, color='blue', alpha=0.5, 
                       label=f'Sample of {max_points} points')
        else:
            plt.scatter(self.points[:, 0], self.points[:, 1], s=2, color='blue', alpha=0.5, 
                       label='Points')
        
        # Plot circles
        ax = plt.gca()
        for center in self.circle_centers:
            circle = Circle(center, self.radius, fill=False, edgecolor='red', alpha=0.5)
            ax.add_patch(circle)
            plt.scatter(center[0], center[1], s=15, color='red')
        
        plt.scatter([], [], s=15, color='red', 
                   label=f'{len(self.circle_centers)} circle centers')
        plt.legend()
        plt.title(f'Circle Coverage: {self.n_points} points, radius={self.radius}')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.axis('equal')
        plt.tight_layout()
        
        plt.savefig('circle_coverage_solution.png', dpi=150)
        return plt.gcf()


def generate_clustered_points(n_points, n_clusters, area_size=1000, cluster_std=50, random_ratio=0.2):
    """
    Generate test data with clusters and some random points
    
    Args:
        n_points: total number of points
        n_clusters: number of clusters
        area_size: size of the square area
        cluster_std: standard deviation of clusters
        random_ratio: ratio of random points
        
    Returns:
        numpy array of shape (n_points, 2)
    """
    points = []
    
    # Generate clustered points
    n_clustered = int(n_points * (1 - random_ratio))
    points_per_cluster = n_clustered // n_clusters
    
    for _ in range(n_clusters):
        # Random cluster center
        center_x = np.random.uniform(100, area_size - 100)
        center_y = np.random.uniform(100, area_size - 100)
        
        # Generate points with normal distribution around center
        cluster_points_x = np.random.normal(center_x, cluster_std, points_per_cluster)
        cluster_points_y = np.random.normal(center_y, cluster_std, points_per_cluster)
        
        # Stack and append
        cluster_points = np.column_stack((cluster_points_x, cluster_points_y))
        points.append(cluster_points)
    
    # Generate random points
    n_random = n_points - n_clustered
    random_points_x = np.random.uniform(0, area_size, n_random)
    random_points_y = np.random.uniform(0, area_size, n_random)
    random_points = np.column_stack((random_points_x, random_points_y))
    points.append(random_points)
    
    # Combine all points
    all_points = np.vstack(points)
    
    # Shuffle
    np.random.shuffle(all_points)
    
    return all_points


def main():
    """Run a demonstration of the algorithm"""
    # Generate test data (50,000 points)
    print("Generating 50,000 points...")
    n_points = 50000
    points = generate_clustered_points(
        n_points=n_points,
        n_clusters=15,
        area_size=1000,
        cluster_std=50,
        random_ratio=0.2
    )
    
    # Set radius
    radius = 30
    print(f"Finding optimal coverage with radius={radius}...")
    
    # Create solver
    solver = CircleCoverage(points, radius)
    
    # Solve
    solution = solver.solve_greedy(use_grid=True, adaptive=True)
    
    # Visualize (on a sample for speed)
    print("Creating visualization...")
    solver.visualize_solution()
    print("Solution saved as 'circle_coverage_solution.png'")


if __name__ == "__main__":
    main()