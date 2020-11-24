import math
import heapq


def _euclidean_dist_loc(map_x: object) -> dict:
    
    """
    Calculates the euclidean distance from any point (x,y) given in map_x object 
    towards its sourrounding neighbour points, which are cointained in map_x.roads.
    
    PSEUDOCODE:
        1. For each node, check sourrounding nodes (connections) from map_x.roads.
            2. Locally store node's x and y coordinates.
            3. Iterate through connections:
                4. For each connection get x and y coordinates.
                5. Calculate distance between node and connection.
                6. Store the distance in local_distance dict as follows:
                    key = node
                    value = dictionary(connected_node : distance)
    
    
    ============== 
    = PARAMETERS =
    ==============
        
        :map_x (object) -> A Map object, containing x,y coordinatates in a .intersections dict()
        :returns: local_distance: A dictionary: {key = position, value = {neighbour : distance}}
        
    """
    
    local_distance = {}
    for node, connections in enumerate(map_x.roads):
        nx, ny = map_x.intersections[node]
        for connection in connections:
            cx, cy = map_x.intersections[connection]
            distance = math.sqrt( (nx-cx)**2 + (ny-cy)**2 )
            local_distance.setdefault(node, {})
            local_distance[node].update( {connection: distance} )
    return local_distance



def _euclidean_dist_glob(map_x: object, goal: int) -> dict:
    
    """
    Calculates the underestimated (minimum) euclidean distance from any point (x,y) 
    given in map_x object towards a specified goal point. Required to guarantee for A*s
    "greedy-best-first-search" paradigm, leading to an expansion of nodes / intersections
    first that are closest by euclidean distance towards the target. 
    Calculates as if distance between point A and goal would be on a straight line
    (minimum possible distance), thus underestimating the true distance, which is important
    for the A* algorithm to suceed
    
    ============== 
    = PARAMETERS =
    ==============
        
        :map_x (object) -> A Map object, containing x,y coordinatates in a .intersections dict()
        :goal (int) -> target position to reach
        
        :returns: global_distance: A dictionary (keys = positions, value = distance)
        
    Note:
        global_distance[goal] returns 0
    """
    
    global_distance = {}
    xg, yg = map_x.intersections[goal]
    for node, (xn, yn) in map_x.intersections.items():
        distance = math.sqrt( (xg-xn)**2 + (yg-yn)**2 )
        global_distance[node] = distance
    return global_distance




def shortest_path(map_x, start=None, goal=None):
    
    """
    A* Algorithm:
    
    ============== 
    = PARAMETERS =
    ==============
        
        :map_x (object) ->  A Map object, containing x,y coordinatates in a .intersections dict()
                            and connected points in an adjacency list in .roads list
        :start (int)    ->  Integer position of where to start the search
        :goal (int)     ->  Integer intersection of where to terminate the search
        
    Note:
        This version of A* is implemented in a way, that it checks all nodes
    
    
    """
    
    if start == goal:
        return [goal]
    
    
    shortest_path = {} # for any neighbour (key) map its closest current node (value)
    geodesic_dist = { key: math.inf for key, _ in map_x.intersections.items() } # for any node store its smallest distance from start
                                                                                # if another node leads to this node with a larger distance ignore it
    
    geodesic_dist[start] = 0
    
    local_distance = _euclidean_dist_loc(map_x)
    global_distance = _euclidean_dist_glob(map_x, goal)
    
    seen = set() # explored states
    frontier = [(geodesic_dist[start], start)] # min-heap, (distance, node)
    
    while frontier:
        
        curr_dist, curr_node = heapq.heappop(frontier)

        if curr_node in seen:
            continue

        seen.add(curr_node)
        
        for neighbour in map_x.roads[curr_node]:
            if neighbour not in seen:
                
                g = geodesic_dist[curr_node] + local_distance[curr_node][neighbour] # geodesic distance travelled so far + distance between curr_node and its neighbour
                h = global_distance[neighbour]                  # heurisitc euclidean underestimated distance from neighbour to goal
                f = g + h                                       # total cost: uniform cost search (g) + greedy best first seach (h)
                
                if g < geodesic_dist[neighbour]:
                    geodesic_dist[neighbour] = g                # update geodesic distances without h (as they are recycled for the whole route)
                                                                # otherwise h would accumulate from all previous nodes
                    heapq.heappush(frontier, (f, neighbour))    # explore the next closest node (lowest f cost) first
                    shortest_path[neighbour] = curr_node        # every time we obtain a lower g, we update the neighbours closest node as curr_node until we reach a minimum == closest node
                    
                else:
                    pass
                    

    
    # Each key in shortest_path is holding a closest intersection / node.
    # Some keys may hold the same closest node.
    # Start node was never a "neighbour" so, calculate back from the end
    
    path = []
    path.append(goal)
    node = goal
    cond = shortest_path[node] == start # condition
    
    while ~ cond == -1: # while condition evaluates to false
        node = shortest_path[node]
        path.append(node)
        cond = shortest_path[node] == start
        
    path.append(start)
    path = list(reversed(path))
    
    return path
