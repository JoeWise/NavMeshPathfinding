

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
			
    #if the points are in the same box, just return the two points as a path
    if source_box == destination_box:
        path.append((source_point, destination_point))
        return path, visited_nodes

    bfspath, prev = bfs(source_box, destination_box, mesh)
    #add the initial point to the path
    path.append((source_point, bfspath[0]))
    for i in range(0,len(bfspath)-1):
        path.append((bfspath[i], bfspath[i+1]))
    #end the path with the destination point
    path.append((bfspath[-1], destination_point))
    
    if path == []:
        print ("no path found!")
    else:
        print ("a path found!")
        # print (str(path))

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