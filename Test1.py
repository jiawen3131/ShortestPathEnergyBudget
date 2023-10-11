import json
import heapq

# Heuristic function for distance between 2 points
def heuristic(nodeA, nodeB, Coord):
    (xA, yA) = Coord[nodeA]
    (xB, yB) = Coord[nodeB]

    return 0  # Manhattan distance

def astar_search(G, Coord, Dist, Cost, start_node, goal_node, energy_budget):

    priority_queue = [(0, start_node, energy_budget)]  # Priority queue to store (cost, node) tuple
    g_scores = {node: float("inf") for node in G}
    f_scores = {node: float("inf") for node in G}
    g_scores[start_node] = 0
    f_scores[start_node] = 0
    visited = set()
    visited_forPath = set()
    
    while priority_queue:
        cost, currentNode, current_budget = heapq.heappop(priority_queue)
        if currentNode == goal_node:
            path = [goal_node]  # Start with the goal node
            while currentNode != start_node:
                for neighbour in G[currentNode]:
                    if g_scores[neighbour] + Dist[(f"{neighbour}, {currentNode}")] == g_scores[currentNode]:
                        visited_forPath.add(currentNode)
                        path.append(neighbour)
                        currentNode = neighbour
                        break
            path.reverse()  # Reverse the path to get it in the correct order
            return path, g_scores[goal_node], energy_budget-current_budget
        
        if currentNode in visited:
            continue

        visited.add(currentNode)
        
        for neighbour in G[currentNode]:
            if current_budget >= Cost[(f"{currentNode}, {neighbour}")]:
                tentative_g_scores = g_scores[currentNode] + Dist[(f"{currentNode}, {neighbour}")]
                if tentative_g_scores < g_scores[neighbour]:
                    g_scores[neighbour] = tentative_g_scores
                    f_scores[neighbour] = g_scores[neighbour] + heuristic(neighbour, goal_node, Coord)
                    updated_budget = current_budget - Cost[(f"{currentNode}, {neighbour}")]
                    heapq.heappush(priority_queue, (f_scores[neighbour], neighbour, updated_budget))
    
    return float("inf"), []  # No path found to the end node

def format_path(path):
    return '->'.join(path)

def main():
    G = {
        "S" : ["1", "2", "3"],
        "1" : ["S", "T"],
        "2" : ["S", "T"],
        "3" : ["S", "T"],
        "T" : ["1", "2", "3"]
    }

    Coord = {
        "S" : [0, 0],
        "1" : [1, 2],
        "2" : [2, 0],
        "3" : [2, -1],
        "T" : [4, 0]
    }

    Dist = {
        "S, 1" : 4,
        "1, S" : 4,
        "S, 2" : 2,
        "2, S" : 2,
        "S, 3" : 4,
        "3, S" : 4,
        "1, T" : 8,
        "T, 1" : 8,
        "2, T" : 8,
        "T, 2" : 8,
        "3, T" : 12,
        "T, 3" : 12,
    }

    Cost = {
        "S, 1" : 7,
        "1, S" : 7,
        "S, 2" : 6,
        "2, S" : 6,
        "S, 3" : 3,
        "3, S" : 3,
        "1, T" : 3,
        "T, 1" : 3,
        "2, T" : 6,
        "T, 2" : 6,
        "3, T" : 2,
        "T, 3" : 2,
    }

    start_node = "S"
    goal_node = "T"
    energy_budget = 11

    # Perform A* search 
    shortest_path, shortest_path_cost, total_energy_cost = astar_search(G, Coord, Dist, Cost, start_node, goal_node, energy_budget)

    if shortest_path_cost == float("inf"):
        print("No path found from node A to node T.")
    else:
        formatted_path = format_path(shortest_path)
       
        print(f"Shortest path: {formatted_path}.")
        print(f"Shortest distance: {shortest_path_cost}.")
        print(f"Total energy cost: {total_energy_cost}.")

if __name__ == "__main__":
    main()