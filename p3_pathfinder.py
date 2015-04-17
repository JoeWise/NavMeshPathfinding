

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

    bfspath, prev = bfs(source_box, destination_box, mesh)

    for i in range(0,len(bfspath)-1):
        point1 = ((bfspath[i][0] + bfspath[i][1])/2, (bfspath[i][2] + bfspath[i][3])/2)
        point2 = ((bfspath[i+1][0] + bfspath[i+1][1])/2, (bfspath[i+1][2] + bfspath[i+1][3])/2)

        path.append((point1, point2))

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
                q.append(next_node)

    if node == destination_box:
        path = []
        while node:
            path.append(node)
            node = prev[node]
        path.reverse()
        return path, prev
    else:
        return [], prev

    # visit node
    # visit all of the nodes children
    # when finished visit all of children's children