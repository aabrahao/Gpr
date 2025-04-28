# ChatGPT

import numpy as np
from scipy import spatial
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# Stage 1: Grid-based initial clustering
def grid_based_initial_clustering(points, radius):
    grid_size = radius * np.sqrt(2)
    grid_centers = {}

    for point in points:
        grid_x = int(point[0] // grid_size)
        grid_y = int(point[1] // grid_size)
        key = (grid_x, grid_y)
        if key not in grid_centers:
            grid_centers[key] = []
        grid_centers[key].append(point)

    initial_circles = [np.mean(pts, axis=0) for pts in grid_centers.values()]
    return np.array(initial_circles)

# Greedy algorithm with overlap constraints
def greedy(x, y, radius, overlap):
    points = np.column_stack((x, y))
    tree = spatial.KDTree(points)
    covered = np.zeros(len(points), dtype=bool)
    circles = []

    while not np.all(covered):
        uncovered_indices = np.flatnonzero(~covered)
        uncovered_points = points[uncovered_indices]
        neighbors_list = tree.query_ball_point(uncovered_points, r=radius)
        coverage_counts = np.array([np.sum(~covered[indices]) for indices in neighbors_list])
        best_idx = np.argmax(coverage_counts)
        best_center = uncovered_points[best_idx]
        circles.append(best_center)
        covered[neighbors_list[best_idx]] = True

    circles = np.array(circles).reshape(-1, 2)
    return circles[:, 0], circles[:, 1]

# Stage 2: Refinement
def refine_circles(points, initial_circles, radius, overlap):
    circle_tree = spatial.KDTree(initial_circles)
    covered = np.zeros(len(points), dtype=bool)

    neighbors = circle_tree.query_ball_point(points, radius)
    for idx, neighbor in enumerate(neighbors):
        if neighbor:
            covered[idx] = True

    uncovered_points = points[~covered]
    xc, yc = greedy(uncovered_points[:, 0], uncovered_points[:, 1], radius, overlap)

    all_xc = np.concatenate([initial_circles[:, 0], xc])
    all_yc = np.concatenate([initial_circles[:, 1], yc])

    return all_xc, all_yc

# Integrated Two-stage algorithm
def two_stage_covering_algorithm(x, y, radius, overlap):
    points = np.column_stack((x, y))
    initial_circles = grid_based_initial_clustering(points, radius)
    xc, yc = refine_circles(points, initial_circles, radius, overlap)
    return xc, yc, initial_circles[:, 0], initial_circles[:, 1]

# Visualization function
def visualize(x, y, xc, yc, initial_xc, initial_yc, radius):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(x, y, c='blue', s=10, label='Points', alpha=0.5)

    for cx, cy in zip(initial_xc, initial_yc):
        circle = Circle((cx, cy), radius, fill=True, color='green', alpha=0.3)
        ax.add_patch(circle)

    for cx, cy in zip(xc, yc):
        circle = Circle((cx, cy), radius, fill=False, edgecolor='red', linewidth=1.5, alpha=0.7)
        ax.add_patch(circle)

    ax.set_title('Two-stage Coverage Algorithm')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend(['Points', 'Stage 1 circles', 'Stage 2 refined circles'])
    ax.axis('equal')
    plt.grid(True)
    plt.show()

# Example usage
if __name__ == '__main__':
    np.random.seed(42)
    x = np.random.uniform(0, 100, 1000)
    y = np.random.uniform(0, 100, 1000)
    radius = 5
    overlap = 0.2

    final_xc, final_yc, initial_xc, initial_yc = two_stage_covering_algorithm(x, y, radius, overlap)

    visualize(x, y, final_xc, final_yc, initial_xc, initial_yc, radius)
