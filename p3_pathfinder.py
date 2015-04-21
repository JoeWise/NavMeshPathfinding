from math import sqrt
from heapq import heappush, heappop


def point_distance(point1, point2):
    return sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def find_path(source_point, destination_point, mesh):
    path = []
    visited_nodes = []
    source_box = ()
    destination_box = ()

    boxes = mesh['boxes']
    for box in boxes:
        if box[0] < source_point[0] < box[1] and box[2] < source_point[1] < box[3]:
            source_box = box
            visited_nodes.append(box)
        if box[0] < destination_point[0] < box[1] and box[2] < destination_point[1] < box[3]:
            destination_box = box
            visited_nodes.append(box)

    #check to make sure the selected point have boxes to search with (didn't click a wall)
    if source_box == () or destination_box == ():
        print ("no path found!")
        return path, visited_nodes
			
    #if the points are in the same box, just return the two points as a path
    if source_box == destination_box:
        path.append((source_point, destination_point))
        return path, visited_nodes

    #loaded_path, prev = bfs(source_box, destination_box, mesh)
    #loaded_path, prev = dijkstras(source_point, destination_point, mesh)
    loaded_path, prev = a_star(source_point, destination_point, mesh)
    #add the initial point to the path
    if loaded_path == []:
        print ("no path found!")
    else:
        path.append((source_point, loaded_path[0]))
        for i in range(0,len(loaded_path)-1):
            path.append((loaded_path[i], loaded_path[i+1]))
        #end the path with the destination point
        path.append((loaded_path[-1], destination_point))

    for node in prev:
        if prev[node] and not isinstance(prev[node], (int, long)):
            visited_nodes.append(prev[node])

    return path, visited_nodes


def bfs(source_box, destination_box, mesh):

    prev = {source_box: None}
    detail_points = {}

    q = [source_box]

    node = source_box

    while q:
        node = q.pop()

        if node == destination_box:
            break

        adj = mesh['adj']
        neighbors = adj[node]
        for next_node in neighbors:
            if next_node not in prev:
                prev[next_node] = node
                #create the point that we will be moving to
                x1 = max(next_node[0], node[0])
                x2 = min(next_node[1], node[1])
                y1 = max(next_node[2], node[2])
                y2 = min(next_node[3], node[3])
                point = ((x1 + x2)/2, (y1 + y2)/2)
                detail_points[next_node] = point
                q.append(next_node)

    if node == destination_box:
        path = []
        while node:
            if prev[node]:
                path.append(detail_points[node])
            node = prev[node]
        path.reverse()
        return path, prev
    else:
        return [], prev

    # visit node
    # visit all of the nodes children
    # when finished visit all of children's children

def dijkstras(source_point, destination_point, mesh):

    #first find the source and destination boxes
    source_box = ()
    destination_box = ()

    boxes = mesh['boxes']
    for box in boxes:
        if box[0] < source_point[0] < box[1] and box[2] < source_point[1] < box[3]:
            source_box = box
        if box[0] < destination_point[0] < box[1] and box[2] < destination_point[1] < box[3]:
            destination_box = box

    prev = {source_box: None}
    detail_points = {}
    dist = {}

    detail_points[source_box] = source_point
    dist[source_point] = 0
    q = [(0, source_box)]

    #nodes will contain their distance to make heappop pop the next shortest node
    node = (0, source_box)

    while q:
        node = heappop(q)

        #Check if node is found
        if node[1] == destination_box:
            path = []
            check_node = node[1]
            while check_node:
                if prev[check_node]:
                    path.append(detail_points[check_node])
                check_node = prev[check_node]
            path.reverse()
            return path, prev

        adj = mesh['adj']
        neighbors = adj[node[1]]
        for next_node in neighbors:
            if next_node not in prev:
                prev[next_node] = node[1]
                #create the point that we will be moving to
                x1 = max(next_node[0], node[1][0])
                x2 = min(next_node[1], node[1][1])
                y1 = max(next_node[2], node[1][2])
                y2 = min(next_node[3], node[1][3])
                point = ((x1 + x2)/2, (y1 + y2)/2)
                prev_point = detail_points[node[1]]
                distance = dist[detail_points[node[1]]] + point_distance(prev_point, point)
                detail_points[next_node] = point
                dist[point] = distance
                heappush(q, (distance, next_node))

    return [], prev

def a_star(source_point, destination_point, mesh):

    #first find the source and destination boxes
    source_box = ()
    destination_box = ()

    boxes = mesh['boxes']
    for box in boxes:
        if box[0] < source_point[0] < box[1] and box[2] < source_point[1] < box[3]:
            source_box = box
        if box[0] < destination_point[0] < box[1] and box[2] < destination_point[1] < box[3]:
            destination_box = box

    prev = {source_box: None}
    detail_points = {}
    dist = {}

    start_dist = point_distance(source_point, destination_point)

    detail_points[source_box] = source_point
    dist[source_point] = 0
    #nodes will contain their distance to make heappop pop the next shortest node
    q = [(start_dist, source_box)]

    while q:
        node = heappop(q)

        #Check if node is found
        if node[1] == destination_box:
            path = []
            check_node = node[1]
            while check_node:
                if prev[check_node]:
                    path.append(detail_points[check_node])
                check_node = prev[check_node]
            path.reverse()
            return path, prev

        adj = mesh['adj']
        neighbors = adj[node[1]]
        for next_node in neighbors:
            if next_node not in prev:
                prev[next_node] = node[1]
                #create the point that we will be moving to
                x1 = max(next_node[0], node[1][0])
                x2 = min(next_node[1], node[1][1])
                y1 = max(next_node[2], node[1][2])
                y2 = min(next_node[3], node[1][3])
                point = ((x1 + x2)/2, (y1 + y2)/2)
                prev_point = detail_points[node[1]]
                distance = dist[detail_points[node[1]]] + point_distance(prev_point, point)
                detail_points[next_node] = point
                dist[point] = distance
                dist_remaining = point_distance(point, destination_point)
                heappush(q, (distance + dist_remaining, next_node))

    return [], prev
