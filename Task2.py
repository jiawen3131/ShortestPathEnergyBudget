import json
import heapq

# Function for performing modified Uniform Cost Search with an energy constraint.
def modified_uniform_cost_search(G, Coord, Dist, Cost, start_node, end_node, energy_constraint):
    """
    Perform Uniform Cost Search (UCS) with an energy constraint.

    Args:
        G (dict): Graph representation with nodes and their neighbors.
        Coord (dict): Coordinate information for nodes.
        Dist (dict): Distance between node pairs.
        Cost (dict): Energy cost associated with edges.
        start_node (str): Starting node.
        end_node (str): Goal node.
        energy_constraint (int): Maximum energy budget.

    Returns:
        explored_nodes (int): Number of nodes explored during the search.
        shortest_solution (tuple): Tuple containing path, distance, and total energy cost.
    """
    # Initialize the priority queue, visited set, and exploration counter.
    # Start from the start node with initial values of 0.
    priority_queue = [(0, 0, 0, start_node, [])]
    
    # Create a set to keep track of visited nodes.
    visited = set()

    # Counter to track the number of explored nodes.
    explored_nodes = 0

    # Initialize the variable to store the shortest solution.
    shortest_solution = None

    # Main search loop.
    while priority_queue:

        # Pop the node with the lowest combined cost.
        combined_cost, distance, total_energy_cost, node, path = heapq.heappop(priority_queue)

        # Check if the current node is the end node.
        if node == end_node:
            # Found a path to the end node.
            if shortest_solution is None or distance < shortest_solution[1]:
                shortest_solution = (path, distance, total_energy_cost)

        # Check if the node has been visited.
        if node in visited:
            continue  # Skip nodes that have already been visited
        
        # Mark the current node as visited and increment the exploration counter.
        visited.add(node)
        explored_nodes += 1

        # Explore neighbors of the current node.
        for neighbor in G[node]:
            neighbor_distance = Dist[f"{node},{neighbor}"]
            new_energy_cost = total_energy_cost + Cost[f"{node},{neighbor}"]
            new_distance = distance + neighbor_distance

            # Check if the new energy cost is within the energy constraint.
            if new_energy_cost <= energy_constraint:

                # Calculate the new combined cost, considering both distance and energy cost.
                combined_cost = new_distance + total_energy_cost*0.05

                # Add the neighbor to the priority queue for further exploration.
                heapq.heappush(priority_queue, (combined_cost, new_distance, new_energy_cost, neighbor, path + [node]))

    # Return the number of explored nodes and the shortest solution.
    return explored_nodes, shortest_solution

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

# Main function to execute the search.
def main():
    # Load data from JSON files.
    with open('Coord.json', 'r') as coord_file:
        Coord = json.load(coord_file)

    with open('G.json', 'r') as cost_file:
        G = json.load(cost_file)

    with open('Dist.json', 'r') as dist_file:
        Dist = json.load(dist_file)

    with open('Cost.json', 'r') as cost_file:
        Cost = json.load(cost_file)

    start_node = "1"
    end_node = "50"
    energy_constraint = 287932

    # Perform modified UCS with energy constraint.
    explored_nodes, shortest_solution = modified_uniform_cost_search(G, Coord, Dist, Cost, start_node, end_node, energy_constraint)

    # Print header for the task.
    print("*"*10 + " Task 2 - Uniformed Cost Search with Energy Constraint of 287932 " + "*"*10)
    
    if shortest_solution is None:
        print(f"No path found from node 1 to node 50 within the energy constraint of {energy_constraint}.")
    else:
        formatted_path = format_path(shortest_solution[0] + [end_node])
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
        print(f"Number of explored nodes: {explored_nodes}")
        print()
    
    # Print footer.
    print("*"*30)

if __name__ == "__main__":
    main()