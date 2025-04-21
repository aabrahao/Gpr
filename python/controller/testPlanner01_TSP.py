from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import numpy as np
import matplotlib.pyplot as plt

import Numeric as nm

#def euclidian_distance_matrix(x,y,scale = 1000):
#    n = len(x)
#    matrix = np.zeros((n,n))
#    for i in range(n):
#        for j in range(n):
#            matrix[i][j] = nm.distance(x[i],y[i],x[j],y[j])
#    matrix = scale*matrix
#    matrix = matrix.astype(int)
#    return matrix

def euclidian_distance_matrix(x, y):
    dx = x[:, None] - x[None, :]
    dy = y[:, None] - y[None, :]
    return np.sqrt(dx**2 + dy**2)

def solve(x, y, calculate_distance_matrix = euclidian_distance_matrix, open_ended = False):
    n = len(x)
    # Solver    
    if open_ended:
        manager = pywrapcp.RoutingIndexManager(n, 1, [0], [n-1])
    else:
        manager = pywrapcp.RoutingIndexManager(n, 1, 0)
    routing = pywrapcp.RoutingModel(manager)
    # Set distant cost function
    distance_matrix = 1000*calculate_distance_matrix(x,y)
    distance_matrix = distance_matrix.astype(int)
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]
    # Set callback
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    # Set the search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 30  # 30 seconds time limit
    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)
    # Process solution
    if solution:
        index = routing.Start(0)
        nodes = []
        # Add nodes
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            nodes.append(node)
            index = solution.Value(routing.NextVar(index))
        # Add final node
        node = manager.IndexToNode(index)
        nodes.append(node)
        # Numpy
        return np.array(nodes).astype('int')
    else:
        return np.array([]).astype('int')

def route(x,y,nodes):
    return x[nodes], y[nodes]

def display(x,y,nodes):
    if len(nodes):
        print(f'Route: {nodes}')
        x,y = route(x,y,nodes)
        l = nm.length(x,y)
        print(f'Distance: {l}')
        print(f'Open ended: {nodes[0]!=nodes[-1]} ')
    else:
        print('Ops, no solution found!')

def plot(x,y,nodes,pause=0.5):
        xr,yr = route(x,y,nodes)
        plt.figure(figsize=(10, 8))
        # Nodes
        plt.scatter(x, y, c='black', s=50)
        plt.scatter(xr[0], yr[0], c='green', s=100, label='Start')
        plt.scatter(xr[-1], yr[-1], c='red', s=100, label='End')
        # Decoration
        if nodes[0] == nodes[-1]:
            plt.title('TSP Solution')
        else:
            plt.title('Open-Ended TSP Solution')
        plt.legend()
        plt.grid(True)
        # Labels
        for i, (xa, ya) in enumerate(zip(x, y)):
            plt.annotate(str(i), (xa, ya), xytext=(5, 5), textcoords='offset points')
        plt.pause(pause)
        # Route
        for i in range(len(x)-1):
            plt.plot((xr[i],xr[i+1]), (yr[i],yr[i+1]), 'b-', linewidth=1)
            plt.pause(pause)
        plt.show()

def main():
    n = 20
    x = 100*nm.random(n)
    y = 100*nm.random(n)
    
    nodes = solve(x,y,open_ended=True)
    
    display(x, y, nodes)
    plot(x, y, nodes)

if __name__ == "__main__":
    main()