import json
import heapq
import math

# Heuristic function for distance between 2 points
def heuristic(nodeA, goal_node, Coord, Cost, energy_budget):
    (xA, yA) = Coord[nodeA]
    (xB, yB) = Coord[goal_node]
    remaining_distance = abs(xA - xB) + abs(yA - yB) # Manhattan distance
    euclidean_distance = math.sqrt((xA - xB)**2 + (yA - yB)**2)
    remaining_cost = energy_budget * remaining_distance

    return euclidean_distance
    # return remaining_distance + remaining_cost
    # the closer the heuristic to the actual minimum cost of the shortest path, lesser iteration, hence more efficient
    # is it admissible?

def astar_search(G, Coord, Dist, Cost, start_node, goal_node, energy_budget):

    priority_queue = [(0, start_node, energy_budget)]  # Priority queue to store (cost, node) tuple
    g_scores = {node: float("inf") for node in G}
    f_scores = {node: float("inf") for node in G}
    g_scores[start_node] = 0
    f_scores[start_node] = 0
    visited = set()
    visited_forPath = set()
    expanded = 0

    while priority_queue:
        cost, currentNode, current_budget = heapq.heappop(priority_queue)

        if currentNode == goal_node:
            # print("enter last stage")
            if currentNode == goal_node:
                path = [goal_node]  # Start with the goal node
                while currentNode != start_node:
                    for neighbour in G[currentNode]:
                        if g_scores[neighbour] + Dist[f"{neighbour},{currentNode}"] == g_scores[currentNode]:
                            visited_forPath.add(currentNode)
                            path.append(neighbour)
                            currentNode = neighbour
                            break
                path.reverse()  # Reverse the path to get it in the correct order

            print(f"nodes expanded = {expanded}")
            return path, g_scores[goal_node], energy_budget-current_budget
        
        if currentNode in visited:
            continue

        visited.add(currentNode)
        expanded += 1
        # print(f"node visited = {currentNode}")
        
        for neighbour in G[currentNode]:
            if current_budget >= Cost[(f"{currentNode},{neighbour}")]:
                tentative_g_scores = g_scores[currentNode] + Dist[f"{currentNode},{neighbour}"]
                if tentative_g_scores < g_scores[neighbour]:
                    g_scores[neighbour] = tentative_g_scores
                    f_scores[neighbour] = g_scores[neighbour] + heuristic(neighbour, goal_node, Coord, Cost, energy_budget)
                    heapq.heappush(priority_queue, (f_scores[neighbour], neighbour, current_budget - Cost[(f"{currentNode},{neighbour}")]))
    
    return [], float("inf"), float("inf")  # No path found to the end node

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
    shortest_path, shortest_path_cost, total_energy_cost = astar_search(G, Coord, Dist, Cost, start_node, goal_node, energy_budget)

    if shortest_path_cost == float("inf"):
        print("No path found from node 1 to node 50.")
    else:
        formatted_path = format_path(shortest_path)
       
        print(f"Shortest path: {formatted_path}.")
        print(f"Shortest distance: {shortest_path_cost}.")
        print(f"Total energy cost: {total_energy_cost}.")

if __name__ == "__main__":
    main()