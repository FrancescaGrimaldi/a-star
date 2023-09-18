from Map import Map_Obj
from queue import PriorityQueue

def heuristic(a, b):

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

def alg(start, goal, map):

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

    frontier = PriorityQueue()
    frontier.put(start, heuristic(goal, start))

    came_from = {}
    cost_so_far = {}
    came_from[tuple(start)] = None
    cost_so_far[tuple(start)] = 0

    while not frontier.empty():
        current = frontier.get()

        # if the current node is the goal, we have found the shortest path and we can stop our search
        if current == goal:
            break

        for next in neighbors(map, current):
            new_cost = cost_so_far[tuple(current)] + map.get_cell_value(next)
            if tuple(next) not in cost_so_far or new_cost < cost_so_far[tuple(next)]:
                cost_so_far[tuple(next)] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[tuple(next)] = tuple(current)
    
    return came_from, cost_so_far

def neighbors(map, pos):

    """
    Finds the neighbors of a given node.

    Parameters
    ----------
    map : Map_Obj
        The map object representing the map
    pos : list[int, int]
        The coordinates of the node
    
    Returns
    -------
    list[list[int, int], list[int, int], list[int, int], list[int, int]]
        List of the coordinates of the neighbors of the given node.
    """

    neighbors = []
    for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
        string_map = map.get_maps()[1]

        next_node = (pos[0] + new_position[0], pos[1] + new_position[1])
        
        # checking if the next node is out of bounds
        if next_node[0] > (string_map.shape[0] - 1) or next_node[0] < 0 or next_node[1] > (string_map.shape[1] - 1) or next_node[1] < 0:
            continue

        # checking if the next node is a wall
        if map.get_cell_value(next_node) != -1 :
            neighbors.append(next_node)

    return neighbors

def find_path(map):

    """
    Finds the shortest path between the start and goal positions of the given map,
    and prints the map with the path highlighted.

    Parameters
    ----------
    map : Map_Obj
        The map object representing the map
    """

    # getting the start and goal positions
    start = map.get_start_pos()
    goal = map.get_goal_pos()

    came_from, cost_so_far = alg(start, goal, map)  # calling the A* algorithm

    node = tuple(goal)      # starting from the goal node and going backwards to the start
    while came_from[tuple(node)] != tuple(start):
        map.set_cell_value(came_from[tuple(node)], ";")
        node = came_from[tuple(node)]
    
    map.show_map()
    #print('COST OF GOAL')
    #print(cost_so_far[tuple(goal)])

def main():
    map = Map_Obj(4)
    map.show_map()
    find_path(map)

if __name__ == "__main__":
    main()