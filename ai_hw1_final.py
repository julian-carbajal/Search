

roadmap = dict(
    Arad=dict(Zerind=75, Sibiu=140, Timisoara=118),
    Bucharest=dict(Urziceni=85, Pitesti=101, Giurgiu=90, Fagaras=211),
    Craiova=dict(Drobeta=120, Rimnicu=146, Pitesti=138),
    Drobeta=dict(Craiova=120, Mehadia=75),
    Eforie=dict(Hirsova=86),
    Fagaras=dict(Bucharest=211, Sibiu=99),
    Giurgiu=dict(Bucharest=90),
    Hirsova=dict(Eforie=86, Urziceni=98),
    Iasi=dict(Vaslui=92, Neamt=87),
    Lugoj=dict(Timisoara=111, Mehadia=70),
    Mehadia=dict(Lugoj=70, Drobeta=75),
    Neamt=dict(Iasi=87),
    Oradea=dict(Zerind=71, Sibiu=151),
    Pitesti=dict(Bucharest=101, Rimnicu=97, Craiova=138),
    Rimnicu=dict(Craiova=146, Sibiu=80, Pitesti=97),
    Sibiu=dict(Arad=140, Fagaras=99, Oradea=151, Rimnicu=80),
    Timisoara=dict(Arad=118,Lugoj=111),
    Vaslui=dict(Iasi=92, Urziceni=98),
    Urziceni=dict(Vaslui=142, Bucharest=85, Hirsova=98),
    Zerind=dict(Arad=75, Oradea=71))

# Demonstration of how to use this data structure
#  by printing the map elements and performing a sanity check -
# Please remove or comment out this section in your final submission
for city, neighbors in roadmap.items():
  print(city, neighbors)
  for next_city, distance in neighbors.items():
    print("  The distance from %s to %s is %d" % (city, next_city, distance))
    if next_city not in roadmap:
      print(" ----- ERROR! %s is not in the city node list -----" % next_city)

"""**Your code starts below.  Please organized them in multiple cells with a formatted text cell before each code cell to explain the purpose of the code cell**

# **CODE LOGIC**
"""

from collections import deque
import time
import tracemalloc


# Heuristic values - straight-line distances to Bucharest
heuristic = {
    'Arad': 366,
    'Bucharest': 0,
    'Craiova': 160,
    'Drobeta': 242,
    'Eforie': 161,
    'Fagaras': 176,
    'Giurgiu': 77,
    'Hirsova': 151,
    'Iasi': 226,
    'Lugoj': 244,
    'Mehadia': 241,
    'Neamt': 234,
    'Oradea': 380,
    'Pitesti': 100,
    'Rimnicu': 193,
    'Sibiu': 253,
    'Timisoara': 329,
    'Urziceni': 80,
    'Vaslui': 199,
    'Zerind': 374
}

def reconstruct_path(came_from, current):
    """Reconstruct the path from start to goal"""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]  # Reverse the path to get from start to goal

def calculate_path_cost(path):
    """Calculate the total cost of a path"""
    total_cost = 0
    for i in range(len(path) - 1):
        total_cost += roadmap[path[i]][path[i + 1]]
    return total_cost

def print_path(path, total_cost):
    """Print the solution path and its cost"""
    path_str = " -> ".join(path)
    print(f"Solution path: {path_str}")
    print(f"Total path cost: {total_cost}")

def breadth_first_search(start, goal):
    """Implement breadth-first search algorithm"""
    print("\n===== BREADTH-FIRST SEARCH =====")

    # Performance tracking
    tracemalloc.start()
    start_time = time.time()

    # Initialize the frontier with the start node
    frontier = deque([(start, None, 0)])  # (city, parent, cost)
    frontier_set = {start}  # For efficient membership checking

    # Initialize the explored set
    explored = set()

    # Keep track of parent nodes for path reconstruction
    came_from = {}

    step = 0

    while frontier:
        step += 1
        print(f"\nStep {step}:")

        # Get the next node from the frontier (FIFO)
        current, parent, cost = frontier.popleft()
        frontier_set.remove(current)

        # Print the node being expanded
        if parent:
            print(f"Expanding: {parent}->{current}({cost})")
        else:
            print(f"Expanding: {current}(0)")

        # Check if we've reached the goal
        if current == goal:
            # Reconstruct the path
            came_from[current] = parent
            path = reconstruct_path(came_from, current)
            total_cost = calculate_path_cost(path)

            # Print the solution
            print_path(path, total_cost)

            # Performance metrics
            end_time = time.time()
            current, peak = tracemalloc.get_traced_memory()
            tracemalloc.stop()

            return {
                "path": path,
                "cost": total_cost,
                "time": end_time - start_time,
                "space": peak / 1024,  # KB
                "nodes_expanded": step
            }

        # Add the current node to the explored set
        explored.add(current)

        # Add the parent relationship for path reconstruction
        if parent:
            came_from[current] = parent

        # Expand the current node
        neighbors = []
        for neighbor, distance in roadmap[current].items():
            if neighbor not in explored and neighbor not in frontier_set:
                new_cost = cost + distance
                neighbors.append((neighbor, current, new_cost))
                frontier.append((neighbor, current, new_cost))
                frontier_set.add(neighbor)

        # Print the current frontier
        print("Frontier:")
        if frontier:
            for city, parent, cost in frontier:
                if parent:
                    print(f"  {parent}->{city}({cost})")
                else:
                    print(f"  {city}(0)")
        else:
            print("  Empty")

    # If we exhaust the frontier without finding the goal
    print("No solution found.")

    # Performance metrics
    end_time = time.time()
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    return {
        "path": None,
        "cost": float('inf'),
        "time": end_time - start_time,
        "space": peak / 1024,  # KB
        "nodes_expanded": step
    }

def depth_first_search(start, goal):
    """Implement depth-first search algorithm"""
    print("\n===== DEPTH-FIRST SEARCH =====")

    # Performance tracking
    tracemalloc.start()
    start_time = time.time()

    # Initialize the frontier with the start node (using a stack for DFS)
    frontier = [(start, None, 0)]  # (city, parent, cost)
    frontier_set = {start}  # For efficient membership checking

    # Initialize the explored set
    explored = set()

    # Keep track of parent nodes for path reconstruction
    came_from = {}

    step = 0

    while frontier:
        step += 1
        print(f"\nStep {step}:")

        # Get the next node from the frontier (LIFO)
        current, parent, cost = frontier.pop()
        frontier_set.remove(current)

        # Print the node being expanded
        if parent:
            print(f"Expanding: {parent}->{current}({cost})")
        else:
            print(f"Expanding: {current}(0)")

        # Check if we've reached the goal
        if current == goal:
            # Reconstruct the path
            came_from[current] = parent
            path = reconstruct_path(came_from, current)
            total_cost = calculate_path_cost(path)

            # Print the solution
            print_path(path, total_cost)

            # Performance metrics
            end_time = time.time()
            current, peak = tracemalloc.get_traced_memory()
            tracemalloc.stop()

            return {
                "path": path,
                "cost": total_cost,
                "time": end_time - start_time,
                "space": peak / 1024,  # KB
                "nodes_expanded": step
            }

        # Add the current node to the explored set
        explored.add(current)

        # Add the parent relationship for path reconstruction
        if parent:
            came_from[current] = parent

        # Expand the current node (in reverse order for DFS to prefer the first neighbors)
        neighbors = []
        for neighbor, distance in sorted(roadmap[current].items(), reverse=True):
            if neighbor not in explored and neighbor not in frontier_set:
                new_cost = cost + distance
                neighbors.append((neighbor, current, new_cost))
                frontier.append((neighbor, current, new_cost))
                frontier_set.add(neighbor)

        # Print the current frontier
        print("Frontier:")
        if frontier:
            for city, parent, cost in frontier:
                if parent:
                    print(f"  {parent}->{city}({cost})")
                else:
                    print(f"  {city}(0)")
        else:
            print("  Empty")

    # If we exhaust the frontier without finding the goal
    print("No solution found.")

    # Performance metrics
    end_time = time.time()
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    return {
        "path": None,
        "cost": float('inf'),
        "time": end_time - start_time,
        "space": peak / 1024,  # KB
        "nodes_expanded": step
    }

def a_star_search(start, goal):
    """Implement A* search algorithm"""
    print("\n===== A* SEARCH =====")

    # Performance tracking
    tracemalloc.start()
    start_time = time.time()

    # Initialize the open set with the start node
    open_set = [(heuristic[start], 0, start, None)]  # (f_score, g_score, city, parent)
    open_set_dict = {start: (0, heuristic[start])}  # city: (g_score, f_score)

    # Initialize the closed set
    closed_set = set()

    # Keep track of parent nodes for path reconstruction
    came_from = {}

    step = 0

    while open_set:
        step += 1
        print(f"\nStep {step}:")

        # Get the node with the lowest f_score
        open_set.sort(reverse=True)  # Sort in descending order to pop from the end
        f_score, g_score, current, parent = open_set.pop()

        # Print the node being expanded
        if parent:
            print(f"Expanding: {parent}->{current}({g_score}, {f_score})")
        else:
            print(f"Expanding: {current}(0, {f_score})")

        # Remove from open_set_dict
        del open_set_dict[current]

        # Check if we've reached the goal
        if current == goal:
            # Reconstruct the path
            came_from[current] = parent
            path = reconstruct_path(came_from, current)
            total_cost = calculate_path_cost(path)

            # Print the solution
            print_path(path, total_cost)

            # Performance metrics
            end_time = time.time()
            current_mem, peak = tracemalloc.get_traced_memory()
            tracemalloc.stop()

            return {
                "path": path,
                "cost": total_cost,
                "time": end_time - start_time,
                "space": peak / 1024,  # KB
                "nodes_expanded": step
            }

        # Add the current node to the closed set
        closed_set.add(current)

        # Add the parent relationship for path reconstruction
        if parent:
            came_from[current] = parent

        # Expand the current node
        for neighbor, distance in roadmap[current].items():
            if neighbor in closed_set:
                continue

            # Calculate the tentative g_score
            tentative_g_score = g_score + distance

            # If this path to neighbor is better than any previous one
            if neighbor not in open_set_dict or tentative_g_score < open_set_dict[neighbor][0]:
                # Calculate the f_score
                f_score = tentative_g_score + heuristic[neighbor]

                # Add to open_set
                if neighbor in open_set_dict:
                    # Remove the old entry
                    open_set = [item for item in open_set if item[2] != neighbor]

                open_set.append((f_score, tentative_g_score, neighbor, current))
                open_set_dict[neighbor] = (tentative_g_score, f_score)

        # Print the current frontier (open set)
        print("Frontier:")
        if open_set:
            # Sort for display purposes
            for f, g, city, parent in sorted(open_set, key=lambda x: x[0]):
                if parent:
                    print(f"  {parent}->{city}({g}, {f})")
                else:
                    print(f"  {city}(0, {f})")
        else:
            print("  Empty")

    # If we exhaust the open set without finding the goal
    print("No solution found.")

    # Performance metrics
    end_time = time.time()
    current_mem, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    return {
        "path": None,
        "cost": float('inf'),
        "time": end_time - start_time,
        "space": peak / 1024,  # KB
        "nodes_expanded": step
    }

def compare_algorithms(results):
    """Compare the performance of the three search algorithms"""
    print("\n===== EXTRA CREDIT ALGORITHM COMPARISON =====")

    print("\nTime Comparison (seconds):")
    for algo, result in results.items():
        print(f"{algo}: {result['time']:.6f} seconds")

    print("\nSpace Usage Comparison (KB):")
    for algo, result in results.items():
        print(f"{algo}: {result['space']:.2f} KB")

    print("\nNodes Expanded Comparison:")
    for algo, result in results.items():
        print(f"{algo}: {result['nodes_expanded']} nodes")

    print("\nPath Cost Comparison:")
    for algo, result in results.items():
        print(f"{algo}: {result['cost']} units")

    print("\nPath Length Comparison:")
    for algo, result in results.items():
        if result['path']:
            print(f"{algo}: {len(result['path'])} nodes - {' -> '.join(result['path'])}")
        else:
            print(f"{algo}: No path found")

    print("\nDiscussion:")
    print("1. Time Efficiency:")
    time_sorted = sorted(results.items(), key=lambda x: x[1]['time'])
    print(f"   - Fastest: {time_sorted[0][0]}")
    print(f"   - Slowest: {time_sorted[-1][0]}")

    print("\n2. Space Efficiency:")
    space_sorted = sorted(results.items(), key=lambda x: x[1]['space'])
    print(f"   - Most Efficient: {space_sorted[0][0]}")
    print(f"   - Least Efficient: {space_sorted[-1][0]}")

    print("\n3. Search Efficiency:")
    nodes_sorted = sorted(results.items(), key=lambda x: x[1]['nodes_expanded'])
    print(f"   - Most Efficient (fewest nodes): {nodes_sorted[0][0]}")
    print(f"   - Least Efficient (most nodes): {nodes_sorted[-1][0]}")

    print("\n4. Path Optimality:")
    cost_sorted = sorted(results.items(), key=lambda x: x[1]['cost'])
    print(f"   - Most Optimal (lowest cost): {cost_sorted[0][0]}")
    print(f"   - Least Optimal (highest cost): {cost_sorted[-1][0]}")

"""# **TASK ONE: Implement a breadth-first search and a depth-first search to find a path from Arad to Bucharest in the Romania roadmap (Figure 1). Print the nodes expanded and the frontier at each step of the search process. The node should be formatted as “Parent_city->Current_city(Path_cost)”. When the solution is found, print the solution path and the total path cost**"""

# Run Breadth-First Search
bfs_result = breadth_first_search("Arad", "Bucharest")

# Run Depth-First Search
dfs_result = depth_first_search("Arad", "Bucharest")

"""#  **TASK TWO: Implement the A* search using the heuristic function in Fig. 2 with the same output requirements, except that the nodes should be formatted as “Parent_city->Current_city(Path_cost, Estimated_total_cost)**"""

# Run A* Search
astar_result = a_star_search("Arad", "Bucharest")

"""# **Extra Credit - algorithm analysis**"""

# Compare Algorithms
results = {
    "BFS": bfs_result,
    "DFS": dfs_result,
    "A*": astar_result
}
compare_algorithms(results)
