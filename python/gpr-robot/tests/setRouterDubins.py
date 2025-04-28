import numpy as np
import matplotlib.pyplot as plt
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# Add n random points
n_random = 50
random_points = np.random.rand(n_random, 2) * 15
points = np.vstack((random_points))
n_points = len(points)

# Euclidean distance function
def distance(i, j):
    return np.linalg.norm(points[i] - points[j])

# Angle penalty for sharp turns
def angle_penalty(a, b, c, threshold=np.pi/3, factor=100.0):
    v1, v2 = points[b] - points[a], points[c] - points[b]
    cosine_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    angle = np.arccos(np.clip(cosine_angle, -1.0, 1.0))
    return factor * max(0, threshold - angle)

# Create OR-Tools TSP solver
def solve_unified_tsp(points, penalty_factor=10.0):
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
    total_penalty = sum(angle_penalty(route[i - 1], route[i], route[i + 1], factor=penalty_factor)
                        for i in range(1, len(route) - 1))

    return route, total_dist + total_penalty

# Solve TSP with angle penalties
route, cost = solve_unified_tsp(points)
print("Optimized Route:", route)
print("Combined Cost:", cost)

# Plot the route
fig, ax = plt.subplots()
ax.plot(*zip(*points[route]), 'ro-', label="Route")

for i, pt in enumerate(points):
    ax.text(pt[0]+0.2, pt[1]+0.2, f'{i}', fontsize=12)

ax.set_title("TSP Route with Turn Penalties")
ax.axis('equal')
ax.grid(True)
plt.legend()
plt.show()
