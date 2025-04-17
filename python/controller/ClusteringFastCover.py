import numpy as np
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