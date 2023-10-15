import json
import heapq
import math

# Function to convert the format -73530767 to 73.530767 and vice versa
def convert_format(value):
    return value / 1_000_000

# Function to calculate the Haversine distance between two coordinates
def haversine_distance(coord1, coord2):
    # Radius of the Earth in kilometers
    earth_radius = 6371.0

    # Convert latitude and longitude from degrees to radians
    lat1 = math.radians(coord1[1])
    lon1 = math.radians(coord1[0])
    lat2 = math.radians(coord2[1])
    lon2 = math.radians(coord2[0])

    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = earth_radius * c

    return distance

# Heuristic function for Haversine distance between 2 points
def heuristic(nodeA, goal_node, Coord):
    coord1 = (convert_format(Coord[nodeA][0]), convert_format(Coord[nodeA][1]))
    coord2 = (convert_format(Coord[goal_node][0]), convert_format(Coord[goal_node][1]))

    h_distance = haversine_distance(coord1, coord2)*1000

    return h_distance

def astar_search(G, Coord, Dist, Cost, start_node, goal_node, energy_budget):
    """
    Perform A* search with an energy constraint.

    Args:
        G (dict): Graph representation with nodes and their neighbors.
        Coord (dict): Coordinate information for nodes.
        Dist (dict): Distance between node pairs.
        Cost (dict): Energy cost associated with edges.
        start_node (str): Starting node.
        goal_node (str): Goal node.
        energy_budget (int): Maximum energy budget.

    Returns:
        explored_nodes (int): Number of nodes explored during the search.
        shortest_solution (tuple): Tuple containing path, distance, and total energy cost.
    """

    # Initialize the priority queue in this order (f_scores, g(n) cumulated shortest distance, cumulated energy cost, start node, path)
    # Start from the start node with initial values of 0.
    priority_queue = [(0, 0, 0, start_node, [])]

    # Set to keep track of visited nodes
    visited = set()  

    # Initialize the variable to store the shortest solution.
    shortest_solution = None

    # Counter to track the number of explored nodes.
    expanded = 0

    # Main search algorithm
    while priority_queue:

        # Pop the node with the lowest f_scores
        f_scores, distance, total_energy_cost, current_node, path = heapq.heappop(priority_queue)

        # Check if the current node is the goal node
        if current_node == goal_node:

            # Found a path to the end node
            if shortest_solution is None or distance < shortest_solution[1]:
                shortest_solution = (path, distance, total_energy_cost)
                # print(f"Nodes visited = {expanded}")

        # Skip nodes that have already been visited
        if current_node in visited:
            continue  

        # Mark the current node as visited and increment the exploration counter.    
        visited.add(current_node)
        expanded += 1

        # Explore the neighbor nodes of the current pop-ed node
        for neighbor in G[current_node]:
            neighbor_distance = Dist[f"{current_node},{neighbor}"]
            energy_cost = Cost[f"{current_node},{neighbor}"]
            new_energy_cost = total_energy_cost + energy_cost
            new_distance = distance + neighbor_distance

            # Check if the cumulated energy cost is within the energy budget
            if new_energy_cost <= energy_budget:

                # Calculate the new f_scores, considerting both distance, energy cost, and heuristic (distance of neighbor to goal node)
                w_energy = 0.05
                w_euclidean = 1
                f_scores = new_distance + w_energy*new_energy_cost + w_euclidean*heuristic(neighbor, goal_node, Coord)
                
                # Push the neighbor node to the priority queue for further exploration 
                heapq.heappush(priority_queue, (f_scores, new_distance, new_energy_cost, neighbor, path + [current_node]))

    return expanded, shortest_solution  # Return the solution with the shortest distance within the energy constraint

# Function to format the path as a string.
def format_path(path):
    """
    Format a list of nodes as a string path.

    Args:
        path (list): List of nodes representing a path.

    Returns:
        formatted_path (str): String representation of the path.
    """
    return '->'.join(path)

def main():
    # Load data from JSON files
    with open('Coord.json', 'r') as coord_file:
        Coord = json.load(coord_file)
    
    with open('G.json', 'r') as cost_file:
        G = json.load(cost_file)
    
    with open('Dist.json', 'r') as dist_file:
        Dist = json.load(dist_file)
    
    with open('Cost.json', 'r') as cost_file:
        Cost = json.load(cost_file)

    start_node = "1"
    goal_node = "50"
    energy_budget = 287932

    # Print header for the task.
    print("*"*10 + " Task 3 - A* Search with Energy Constraint of 287932 " + "*"*10)

    # Perform A* search 
    expanded, shortest_solution = astar_search(G, Coord, Dist, Cost, start_node, goal_node, energy_budget)

    if shortest_solution == None:
        print(f"No path found from node 1 to node 50 within the energy constraint of {energy_budget}.")
    else:
        formatted_path = format_path(shortest_solution[0]+[goal_node])
        shortest_distance = shortest_solution[1]
        total_energy_cost = shortest_solution[2]

        # Print output with formatting.
        print()
        print(f"Shortest path: S->{formatted_path}->T.")
        print()
        print(f"Shortest distance: {shortest_distance}.")
        print()
        print(f"Total energy cost: {total_energy_cost}.")
        print()
        print(f"Number of explored nodes: {expanded}")
        print()

        # Print footer.
        print("*"*73)

if __name__ == "__main__":
    main()