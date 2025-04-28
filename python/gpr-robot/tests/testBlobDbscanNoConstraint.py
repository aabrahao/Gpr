import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from matplotlib.patches import Circle

# DBSCAN-based covering algorithm
def dbscan_circle_covering(x, y, radius):
    points = np.column_stack((x, y))
    
    # DBSCAN clustering with circle radius as eps
    clustering = DBSCAN(eps=radius, min_samples=1).fit(points)
    labels = clustering.labels_

    unique_labels = np.unique(labels)
    circle_centers = []
    for label in unique_labels:
        cluster_points = points[labels == label]
        center = np.mean(cluster_points, axis=0)
        circle_centers.append(center)

    circle_centers = np.array(circle_centers)
    return circle_centers

# Visualization function
def visualize_dbscan_covering(x, y, centers, radius):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(x, y, c='blue', s=10, label='Points', alpha=0.5)

    for center in centers:
        circle = Circle(center, radius, fill=False, edgecolor='red', linewidth=1.5, alpha=0.7)
        ax.add_patch(circle)
        ax.scatter(center[0], center[1], c='red', s=50, alpha=0.9)

    ax.set_title('DBSCAN-based Minimal Circle Covering')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend(['Points', 'Circle centers'])
    ax.axis('equal')
    plt.grid(True)
    plt.show()

# Example usage
if __name__ == '__main__':
    np.random.seed(42)
    x = np.random.uniform(0, 100, 1000)
    y = np.random.uniform(0, 100, 1000)
    radius = 5

    circle_centers = dbscan_circle_covering(x, y, radius)

    print(f'Total circles used: {len(circle_centers)}')

    visualize_dbscan_covering(x, y, circle_centers, radius)
