import numpy as np
import matplotlib.pyplot as plt
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# Adjust time_limit:
#
# Increasing time improves accuracy but at the cost of speed.
#
# Heuristics:
#
# You can experiment with different local_search_metaheuristic methods:
#
#    - routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
#    - routing_enums_pb2.LocalSearchMetaheuristic.TABU_SEARCH
#    - routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING
#
# Large datasets:
#
# For thousands of points, consider heuristics, 
# as exact solutions become computationally expensive.

def toPoints(x,y):
    return np.column_stack((x,y))

def toNumpy(coord):
    return np.array(coord)

def addStartEnd(x,xs,xe):
    return np.insert(np.append(x, xe), 0, xs)

def optimize(x,y,xs,ys,xe,ye):
    
    points = toPoints( addStartEnd(x,xs,xe), addStartEnd(y,ys,ye) )
    #points = toPoints( x, y)
    n = len(points)
    print("Destinations:")
    print(points)

    print('# Calculate Euclidean distance matrix')
    distance_matrix = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)

    print('# OR-Tools TSP setup')
    manager = pywrapcp.RoutingIndexManager(n, 1, [0], [n-1])
    routing = pywrapcp.RoutingModel(manager)
    
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # Scale to integers (required by OR-Tools)
        return int(distance_matrix[from_node][to_node] * 1000)
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    #print('Allow open-ended route by minimizing the "NextVar" at the end node')
    #routing.AddVariableMinimizedByFinalizer(routing.NextVar(routing.End(0)))

    print('Set search parameters for speed and quality')
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.FromSeconds(30)  # set a time limit for fast results

    print('Solve the problem...')
    solution = routing.SolveWithParameters(search_parameters)

    print(' Extract solution')
    if solution:
        route = []
        index = routing.Start(0)
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route.append(points[node_index])
            index = solution.Value(routing.NextVar(index))
        # Return to start (optional)
        # route.append(points[manager.IndexToNode(index)])
        # Output
        print("Optimal route order:")
        for coord in route:
            print(coord)
    else:
        print("Optimal route not found!")
        return np.array([]), np.array([])

    xy = toNumpy(route)
    return xy[:,0], xy[:,1]
