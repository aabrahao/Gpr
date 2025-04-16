import numpy as np
import matplotlib.pyplot as plt
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def toNumpy(coord):
    return np.array(coord)

# Your points (replace with your own coordinates)
points = np.array([
    [0, 0],
    [2, 3],
    [5, 2],
    [6, 6],
    [8, 3],
    [1, 5],
])

num_points = len(points)

# Calculate Euclidean distance matrix
distance_matrix = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)

# OR-Tools TSP setup
manager = pywrapcp.RoutingIndexManager(num_points, 1, 0)  # single vehicle, start at point 0
routing = pywrapcp.RoutingModel(manager)

def distance_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    # Scale to integers (required by OR-Tools)
    return int(distance_matrix[from_node][to_node] * 1000)

transit_callback_index = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# Set search parameters for speed and quality
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
search_parameters.time_limit.FromSeconds(5)  # set a time limit for fast results

# Solve the problem
solution = routing.SolveWithParameters(search_parameters)

# Extract solution
if solution:
    route = []
    index = routing.Start(0)
    while not routing.IsEnd(index):
        node_index = manager.IndexToNode(index)
        route.append(points[node_index])
        index = solution.Value(routing.NextVar(index))
    # Return to start (optional)
    route.append(points[manager.IndexToNode(index)])

    print("Optimal route order:")
    for coord in route:
        print(coord)
else:
    print("No solution found.")

routes = toNumpy(route)
print(routes)

plt.figure()
plt.plot(points[:,0],points[:,1], 'or')
plt.plot(routes[:,0],routes[:,1], '-b')
plt.grid()
plt.show()
