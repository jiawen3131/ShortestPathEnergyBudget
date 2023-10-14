import json
import heapq

def uniform_cost_search_with_energy_cost(G, Coord, Dist, Cost, start_node, end_node, energy_constraint):
    priority_queue = [(0, 0, start_node, [])]  # Priority queue to store (total_energy_cost, cost, node, path) quadruplets
    visited = set()  # Set to keep track of visited nodes
    shortest_solution = None

    while priority_queue:
        total_energy_cost, cost, node, path = heapq.heappop(priority_queue)

        if node == end_node:
            # Found a path to the end node
            if shortest_solution is None or cost < shortest_solution[1]:
                shortest_solution = (path, cost, total_energy_cost)

        if node in visited:
            continue  # Skip nodes that have already been visited

        visited.add(node)

        for neighbor in G[node]:
            edge = f"{node},{neighbor}"
            neighbor_cost = cost + Dist[edge]
            new_energy_cost = total_energy_cost + Cost.get(edge, 0)
            
            if new_energy_cost <= energy_constraint:
                heapq.heappush(priority_queue, (new_energy_cost, neighbor_cost, neighbor, path + [node]))

    return shortest_solution  # Return the solution with the shortest distance and total energy cost

def format_path(path):
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
    end_node = "50"
    energy_constraint = 287932

    # Perform UCS with energy constraint
    shortest_solution = uniform_cost_search_with_energy_cost(G, Coord, Dist, Cost, start_node, end_node, energy_constraint)

    if shortest_solution is None:
        print(f"No path found from node 1 to node 50 within the energy constraint of {energy_constraint}.")
    else:
        formatted_path = format_path(shortest_solution[0])
        shortest_distance = shortest_solution[1]
        total_energy_cost = shortest_solution[2]
       
        print(f"Shortest path: S->{formatted_path}->T.")
        print(f"Shortest distance: {shortest_distance}.")
        print(f"Total energy cost: {total_energy_cost}.")

if __name__ == "__main__":
    main()
