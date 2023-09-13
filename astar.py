from Map import Map_Obj
from queue import PriorityQueue

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
def alg(map):
    start = map.get_start_pos()
    goal = map.get_end_goal_pos()
    frontier = PriorityQueue()
    frontier.put(start, heuristic(goal, start))

    came_from = {}
    cost_so_far = {}
    came_from[tuple(start)] = None
    cost_so_far[tuple(start)] = 0

    while not frontier.empty():
        current = frontier.get()

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
    neighbors = []
    for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
        string_map = map.get_maps()[1]

        next_node = (pos[0] + new_position[0], pos[1] + new_position[1])
        
        if next_node[0] > (string_map.shape[0] - 1) or next_node[0] < 0 or next_node[1] > (string_map.shape[1] - 1) or next_node[1] < 0:
            continue
 
        if map.get_cell_value(next_node) != -1 :
            neighbors.append(next_node)

    return neighbors

def main():
    print('hello')
    map = Map_Obj(1)
    came_from, cost_so_far = alg(map)
    print('CAME_FROM')
    print(came_from)
    print('COST_SO_FAR')
    print(cost_so_far)
    #frontier = PriorityQueue()
    #frontier.put([27,18], 0)
    #frontier.put([40,32], 1)
    #n_s = neighbors(map, frontier.get())
    #print(n_s)
    #n_g = neighbors(map, frontier.get())
    #print(n_g)

main()