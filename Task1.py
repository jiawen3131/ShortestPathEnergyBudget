import json
import heapq

def uniform_cost_search(G, Coord, Dist, start_node, end_node):
    priority_queue = [(0, start_node, [])]  # Priority queue to store (cost, node, path) triplets
    visited = set()  # Set to keep track of visited nodes

    while priority_queue:
        cost, node, path = heapq.heappop(priority_queue)

        if node == end_node:
            return cost, path  # Found the shortest path to the end node

        if node in visited:
            continue  # Skip nodes that have already been visited

        visited.add(node)

        for neighbor in G[node]:
            edge = f"{node},{neighbor}"
            neighbor_cost = cost + Dist[edge]
            heapq.heappush(priority_queue, (neighbor_cost, neighbor, path + [node]))

    return float("inf"), []  # No path found to the end node

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

    # Perform UCS
    shortest_path_cost, shortest_path = uniform_cost_search(G, Coord, Dist, start_node, end_node)

    if shortest_path_cost == float("inf"):
        print("No path found from node 1 to node 50.")
    else:
        formatted_path = format_path(shortest_path)
        total_energy_cost = sum(Cost.get(f"{shortest_path[i]},{shortest_path[i+1]}", 0) for i in range(len(shortest_path)-1))
       
        print(f"Shortest path: S->{formatted_path}->T.")
        print(f"Shortest distance: {shortest_path_cost}.")
        print(f"Total energy cost: {total_energy_cost}.")

if __name__ == "__main__":
    main()