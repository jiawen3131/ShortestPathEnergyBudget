import json
import heapq

def modified_uniform_cost_search(G, Coord, Dist, Cost, start_node, end_node, energy_constraint):
    # Priority queue to store (combined_cost, distance, total_energy_cost, node, path) quintuplets
    priority_queue = [(0, 0, 0, start_node, [])]
    visited = set()  # Set to keep track of visited nodes
    shortest_solution = None

    while priority_queue:
        combined_cost, distance, total_energy_cost, node, path = heapq.heappop(priority_queue)

        if node == end_node:
            # Found a path to the end node
            if shortest_solution is None or distance < shortest_solution[1]:
                shortest_solution = (path, distance, total_energy_cost)

        if node in visited:
            continue  # Skip nodes that have already been visited

        visited.add(node)

        for neighbor in G[node]:
            edge = f"{node},{neighbor}"
            neighbor_cost = Cost.get(edge, 0)
            neighbor_distance = Dist[edge]
            new_energy_cost = total_energy_cost + neighbor_cost
            new_distance = distance + neighbor_distance

            if new_energy_cost <= energy_constraint:
                combined_cost = new_distance + 0.05*new_energy_cost  # You can adjust this cost function to balance distance and energy
                heapq.heappush(priority_queue, (combined_cost, new_distance, new_energy_cost, neighbor, path + [node]))

    return shortest_solution  # Return the solution with the shortest distance within the energy constraint

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

    # Perform modified UCS with energy constraint
    shortest_solution = modified_uniform_cost_search(G, Coord, Dist, Cost, start_node, end_node, energy_constraint)

    if shortest_solution is None:
        print(f"No path found from node 1 to node 50 within the energy constraint of {energy_constraint}.")
    else:
        formatted_path = format_path(shortest_solution[0]+[end_node])
        shortest_distance = shortest_solution[1]
        total_energy_cost = shortest_solution[2]

        print(f"Shortest path: S->{formatted_path}->T.")
        print(f"Shortest distance: {shortest_distance}.")
        print(f"Total energy cost: {total_energy_cost}.")

if __name__ == "__main__":
    main()
