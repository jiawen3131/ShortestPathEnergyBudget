import json
import heapq
import math

# Heuristic function for distance between 2 points
def heuristic(nodeA, goal_node, Coord):
    (xA, yA) = Coord[nodeA]
    (xB, yB) = Coord[goal_node]
    euclidean_distance = math.sqrt((xA - xB)**2 + (yA - yB)**2)

    return euclidean_distance
    # return remaining_distance + remaining_cost
    # the closer the heuristic to the actual minimum cost of the shortest path, lesser iteration, hence more efficient
    # is it admissible?

def astar_search(G, Coord, Dist, Cost, start_node, goal_node, energy_budget):
    priority_queue = [(0, 0, 0, start_node, [])]
    visited = set()  # Set to keep track of visited nodes
    shortest_solution = None
    expanded = 0

    while priority_queue:
        f_scores, distance, total_energy_cost, current_node, path = heapq.heappop(priority_queue)

        if current_node == goal_node:
            # Found a path to the end node
            if shortest_solution is None or distance < shortest_solution[1]:
                shortest_solution = (path, distance, total_energy_cost)
                # print(f"Nodes visited = {expanded}")

        if current_node in visited:
            continue  # Skip nodes that have already been visited

        visited.add(current_node)
        expanded += 1

        for neighbor in G[current_node]:
            neighbor_distance = Dist[f"{current_node},{neighbor}"]
            new_energy_cost = total_energy_cost + Cost[f"{current_node},{neighbor}"]
            new_distance = distance + neighbor_distance

            if new_energy_cost <= energy_budget:
                f_scores = new_distance + 0.05*new_energy_cost + heuristic(neighbor, goal_node, Coord) # You can adjust this cost function to balance distance and energy
                heapq.heappush(priority_queue, (f_scores, new_distance, new_energy_cost, neighbor, path + [current_node]))

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
    goal_node = "50"
    energy_budget = 287932

    # Perform A* search 
    shortest_solution = astar_search(G, Coord, Dist, Cost, start_node, goal_node, energy_budget)

    if shortest_solution == None:
        print("No path found from node 1 to node 50.")
    else:
        formatted_path = format_path(shortest_solution[0]+[goal_node])
        shortest_distance = shortest_solution[1]
        total_energy_cost = shortest_solution[2]

        print(f"Shortest path: {formatted_path}.")
        print(f"Shortest distance: {shortest_distance}.")
        print(f"Total energy cost: {total_energy_cost}.")

if __name__ == "__main__":
    main()