from Map import Map_Obj
from queue import PriorityQueue

def heuristic(a: list[int, int], b: list[int, int]):

    """
    Calculates the heuristic for the A* algorithm: the Manhattan distance between two points.

    Parameters
    ----------
    a : list[int, int]
        The coordinates of the first point
    b : list[int, int]
        The coordinates of the second point

    Returns
    -------
    int
        The Manhattan distance between the given points
    """

    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def neighbors(map: Map_Obj, pos: list[int, int]):

    """
    Finds the walkable neighbors of a given node.

    Parameters
    ----------
    map : Map_Obj
        The map object representing the map
    pos : list[int, int]
        The coordinates of the node
    
    Returns
    -------
    list[list[int, int], list[int, int], list[int, int], list[int, int]]
        List of the coordinates of the walkable neighbors of the given node.
    """

    neighbors = []
    for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:    # north, south, east and west neighbors of the current node
        string_map = map.get_maps()[1]

        next_node = (pos[0] + new_position[0], pos[1] + new_position[1])
        
        # checking if the next node is out of bounds
        if next_node[0] > (string_map.shape[0] - 1) or next_node[0] < 0 or next_node[1] > (string_map.shape[1] - 1) or next_node[1] < 0:
            continue

        # checking if the next node is a wall
        if map.get_cell_value(next_node) != -1 :
            neighbors.append(next_node)

    return neighbors

def alg(start: list[int, int], goal: list[int, int], map: Map_Obj):

    """
    Implementation of the A* algorithm for finding the shortest path between two points.

    Parameters
    ----------
    start : list[int, int]
        The coordinates of the starting point
    goal : list[int, int]
        The coordinates of the goal point
    map : Map_Obj
        The map object representing the map

    Returns
    -------
    tuple[dict, dict]
        Dictionary containing the parent of each node in the shortest path
        Dictionary containing the cost to get to each node in the shortest path
    """

    came_from = {}    # dictionary that contains the parent of each node in the shortest path
    cost_to_node = {}    # dictionary that contains the cost to get to each node in the shortest path

    came_from[tuple(start)] = None    # the start node has no parent
    cost_to_node[tuple(start)] = 0    # the cost to get to the start node is 0

    """
    The frontier is implemented as a priority queue with the priority being the cost to get to the node + the heuristic.
    Every time we pop a node from the frontier, we are guaranteed that it is the one with the lowest cost to get to it.
    """
    frontier = PriorityQueue()
    frontier.put(start, heuristic(goal, start))    # pushing the start node to the frontier

    while not frontier.empty():    # searching until there are no more nodes to explore
        current_node = frontier.get()    # popping the node with the lowest cost to reach it

        # if the current node is the goal, we have found the shortest path and we can stop our search
        if current_node == goal:
            break

        for next_node in neighbors(map, current_node):    # exploring the neighbors of the current node

            # cost to get to the node = cost to get to the parent + cost of the cell itself
            new_cost = cost_to_node[tuple(current_node)] + map.get_cell_value(next_node)
            
            # if the node has not been visited yet or we have found a better path to it, we update its cost and parent and add it to the frontier with its priority
            if tuple(next_node) not in cost_to_node or new_cost < cost_to_node[tuple(next_node)]:
                cost_to_node[tuple(next_node)] = new_cost
                priority = new_cost + heuristic(goal, next_node)    # priority = cost to get to the node + heuristic
                frontier.put(next_node, priority)
                came_from[tuple(next_node)] = tuple(current_node)
    
    return came_from, cost_to_node

def find_path(map: Map_Obj):

    """
    Finds the shortest path between the start and goal positions of the given map,
    and prints the map with the path highlighted in purple.

    Parameters
    ----------
    map : Map_Obj
        The map object representing the map
    """

    # getting the start and goal positions
    start = map.get_start_pos()
    goal = map.get_goal_pos()

    came_from, cost_to_node = alg(start, goal, map)    # calling the A* algorithm

    # drawing the path starting from the goal node and going backwards to the start
    current_node = tuple(goal)
    while came_from[current_node] != tuple(start):
        map.set_cell_value(came_from[current_node], " - ")    # colouring the path
        current_node = came_from[current_node]    # going to the parent node
    
    map.show_map()
    print('Cost to reach the goal:')
    print(cost_to_node[tuple(goal)])


if __name__ == "__main__":
    map = Map_Obj(4)
    map.show_map()
    find_path(map)