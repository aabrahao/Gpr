from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import numpy as np
import matplotlib.pyplot as plt

def solve_open_ended_tsp_with_coordinates(x_coords, y_coords):
    """Solves the open-ended TSP using OR-Tools with x,y coordinates as input."""
    num_nodes = len(x_coords)
    
    # Calculate distance matrix from coordinates
    distance_matrix = np.zeros((num_nodes, num_nodes))
    for i in range(num_nodes):
        for j in range(num_nodes):
            distance_matrix[i][j] = np.sqrt((x_coords[i] - x_coords[j])**2 + 
                                            (y_coords[i] - y_coords[j])**2)
    
    distance_matrix = distance_matrix.astype(int)
    
    # Create the routing model
    # For open-ended TSP, we use separate start and end depots
    manager = pywrapcp.RoutingIndexManager(num_nodes, 1, [0], [num_nodes-1])
    routing = pywrapcp.RoutingModel(manager)
    
    # Define cost function (distance callback)
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    # Set the search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 30  # 30 seconds time limit
    
    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)
    
    # Process solution
    if solution:
        print("Total distance:", solution.ObjectiveValue())
        index = routing.Start(0)
        plan_output = 'Route: '
        route_distance = 0
        
        # For plotting
        route_x = []
        route_y = []
        route_nodes = []
        
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            route_nodes.append(node)
            route_x.append(x_coords[node])
            route_y.append(y_coords[node])
            
            plan_output += f"{node} -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
        
        # Add final node
        node = manager.IndexToNode(index)
        route_nodes.append(node)
        route_x.append(x_coords[node])
        route_y.append(y_coords[node])
        plan_output += f"{node}"
        
        print(plan_output)
        print(f"Route distance: {route_distance}")
        
        # Plot the solution
        plt.figure(figsize=(10, 8))
        
        # Plot all points
        plt.scatter(x_coords, y_coords, c='blue', s=50)
        
        # Highlight start and end points
        plt.scatter(x_coords[0], y_coords[0], c='green', s=100, label='Start')
        plt.scatter(x_coords[route_nodes[-1]], y_coords[route_nodes[-1]], c='red', s=100, label='End')
        
        # Plot the route
        plt.plot(route_x, route_y, 'k-', linewidth=1)
        
        # Add node labels
        for i, (x, y) in enumerate(zip(x_coords, y_coords)):
            plt.annotate(str(i), (x, y), xytext=(5, 5), textcoords='offset points')
        
        plt.title('Open-Ended TSP Solution')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.legend()
        plt.grid(True)
        plt.savefig('tsp_solution.png')
        plt.show()
        
        return route_nodes, route_x, route_y
    else:
        print("No solution found!")
        return None, None, None

# Example usage
# Generate some sample coordinates or replace with your own


def main():
    points = np.array([
        (0, 0), 
        (200, 400), 
        (500, 300), 
        (400, -200),
        (100, -100), 
        (-300, -400), 
        (-400, 100), 
        (-200, 300),
    ])

np.random.seed(42)
num_locations = 15
x_coords = np.random.rand(num_locations) * 100
y_coords = np.random.rand(num_locations) * 100

print("Solving Open-Ended TSP for", num_locations, "locations")
route, route_x, route_y = solve_open_ended_tsp_with_coordinates(x_coords, y_coords)

def main():
    points = np.array([
        (0, 0), 
        (200, 400), 
        (500, 300), 
        (400, -200),
        (100, -100), 
        (-300, -400), 
        (-400, 100), 
        (-200, 300),
    ])
    
    x,y = nm.xy(points)
    route = solve_tsp(x,y)

    print("Optimal visitation sequence:", route)
    plot_route(points, route)

if __name__ == "__main__":
    main()