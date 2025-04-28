
import numpy as np
import matplotlib.pyplot as plt
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import time

# Original Points to visit
points = np.array([(8, 4), (5, 10), (12, 8), (3, 6), (9, 12)])

# Add n random points
n_random = 1000
random_points = np.random.rand(n_random, 2) * 15
points = np.vstack((points, random_points))
n_points = len(points)

# Robot's initial heading (angle in radians)
initial_heading = np.pi / 4  # example 45 degrees
start_point = np.array([0, 0])
points = np.vstack([start_point, points])
n_points += 1

# Euclidean distance function
def distance(i, j):
    return np.linalg.norm(points[i] - points[j])

# Angle penalty focused on short distances, relaxed for long distances
def angle_penalty(a, b, c, threshold=np.pi/3, factor=10000.0):
    dist_ab = distance(a, b)
    dist_bc = distance(b, c)
    segment_length = (dist_ab + dist_bc) / 2
    short_distance_factor = np.exp(-segment_length / 5)  # penalize short segments exponentially more

    v1, v2 = points[b] - points[a], points[c] - points[b]
    cosine_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    angle = np.arccos(np.clip(cosine_angle, -1.0, 1.0))
    return factor * short_distance_factor * max(0, threshold - angle)

# First angle penalty considering the initial robot heading
def initial_angle_penalty(start, next_point, heading, threshold=np.pi/3, factor=30.0):
    initial_direction = np.array([np.cos(heading), np.sin(heading)])
    v = points[next_point] - points[start]
    cosine_angle = np.dot(initial_direction, v) / np.linalg.norm(v)
    angle = np.arccos(np.clip(cosine_angle, -1.0, 1.0))
    short_distance_factor = np.exp(-np.linalg.norm(v) / 5)
    return factor * short_distance_factor * max(0, threshold - angle)

# Create OR-Tools TSP solver
def solve_unified_tsp(points, penalty_factor=30.0):
    manager = pywrapcp.RoutingIndexManager(n_points, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def transit_callback(from_index, to_index):
        from_node, to_node = manager.IndexToNode(from_index), manager.IndexToNode(to_index)
        return int(distance(from_node, to_node) * 1000)

    transit_callback_idx = routing.RegisterTransitCallback(transit_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_idx)

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_params.time_limit.seconds = 10

    solution = routing.SolveWithParameters(search_params)
    route = []
    index = routing.Start(0)
    while not routing.IsEnd(index):
        route.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))

    total_dist = sum(distance(route[i], route[i + 1]) for i in range(len(route) - 1))
    total_penalty = initial_angle_penalty(route[0], route[1], initial_heading, factor=penalty_factor)
    total_penalty += sum(angle_penalty(route[i - 1], route[i], route[i + 1], factor=penalty_factor)
                         for i in range(1, len(route) - 1))

    return route, total_dist + total_penalty

# Solve TSP with angle penalties
route, cost = solve_unified_tsp(points)
print("Optimized Route:", route)
print("Combined Cost:", cost)

# Plot the route step by step with pause
fig, ax = plt.subplots()
ax.plot(points[:, 0], points[:, 1], 'ro', label="Points")
ax.set_title("TSP Route with Animated Step-by-Step Plot")
ax.axis('equal')
ax.grid(True)
plt.legend()

for i in range(len(route) - 1):
    ax.plot(points[route[i:i+2], 0], points[route[i:i+2], 1], 'b-')
    plt.pause(0.5)

plt.show()
