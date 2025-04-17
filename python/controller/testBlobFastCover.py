import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import BallTree

# Efficient algorithm to cover points and return circle centers explicitly
def cover_points_with_circles(x, y, radius):
    points = np.column_stack((x, y))
    tree = BallTree(points)
    num_points = len(points)
    uncovered = set(range(num_points))
    circle_centers = []

    while uncovered:
        idx = uncovered.pop()
        center = points[idx]
        indices = tree.query_radius([center], r=radius)[0]
        uncovered -= set(indices)
        circle_centers.append(center)

    # Verification step
    covered = np.zeros(num_points, dtype=bool)
    for center in circle_centers:
        indices = tree.query_radius([center], r=radius)[0]
        covered[indices] = True

    if not np.all(covered):
        raise ValueError("Error: Some points remain uncovered!")

    xc, yc = np.array(circle_centers).T

    return xc, yc

# Visualization function
def plot_coverage(x, y, xc, yc, radius):
    plt.figure(figsize=(10, 10))
    plt.scatter(x, y, s=10, alpha=0.5, label='Points')

    for center_x, center_y in zip(xc, yc):
        circle = plt.Circle((center_x, center_y), radius, color='blue', fill=False, alpha=0.7)
        plt.gca().add_patch(circle)

    plt.title(f"Coverage of points with {len(xc)} circles")
    plt.axis('equal')
    plt.legend()
    plt.show()

# Example usage:
if __name__ == "__main__":
    np.random.seed(42)
    x = np.random.rand(10000)
    y = np.random.rand(10000)
    radius = 0.05

    xc, yc = cover_points_with_circles(x, y, radius)
    print(f"Covered all points with {len(xc)} circles.")

    plot_coverage(x, y, xc, yc, radius)