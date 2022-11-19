def read_in_edges(edges_file):
    """
    This function reads in the edges from the edges file and creates an adjacency list from it.
    :param edges_file: a file of edges
    :return: adj_list: an adjacency list
    :time complexity: O(V + E) time complexity, where V is the number of vertices and E is the number of edges
    :space complexity: O(V + E) space complexity, where V is the number of vertices and E is the number of edges
    """
    file = open(edges_file)
    adj_list = [[] for _ in range(6105)]
    for line in file:
        toll_road = False
        line = line.strip()
        line = line.split(" ")
        first_vertex = int(line[0])
        second_vertex = int(line[1])
        edge_dist = float(line[2])
        if len(line) == 4:
            toll_road = True
        first_vertex_adj = [second_vertex, edge_dist, toll_road]
        second_vertex_adj = [first_vertex, edge_dist, toll_road]
        adj_list[first_vertex].append(first_vertex_adj)
        adj_list[second_vertex].append(second_vertex_adj)
    file.close()
    return adj_list


def read_in_red_light_camera(vertices_file):
    """
    This function reads in the vertices file and creates a list that has true/false values on where the vertex is a
    red light camera or not.
    :param vertices_file: a file of vertices that are red light cameras
    :return: red_light_list: list of true/false corresponding to whether each vertex is a red light camera or not
    :time complexity: O(V) where V is the number of vertices
    :space complexity: O(V) where V is the number of vertices
    """
    red_light_file = open(vertices_file)
    red_light_list = [False for _ in range(6105)]
    for line in red_light_file:
        line = line.strip()
        red_light_number = int(line)
        red_light_list[red_light_number] = True
    red_light_file.close()
    return red_light_list


def find_shortest_path(source, ad_list, camera_list):
    """
    To find the shortest path to every vertex under the technical condition, it implements Dijkstra’s algorithm using an
    rray-based min-heap that I update to keep tracking of the discovered vertices. Firstly, I create an extra list the
    size of the number of vertices, called vertices list, so that I keep track of where each vertex is located in the
    heap, because it is array based, so I can update nodes in O(1) time. So I first add the source vertex into the min
    heap and then while the min heap is not empty, I get the vertex, v, with the smallest distance, which is located
    at the top of the heap, and remove it from the heap by swapping it with the last node, deleting the min, and sift
    down the node we just swapped into the root. I check if the vertex is a red-light camera or not. If it is a
    red-light camera, I instantly move it to finalised and add it to a list of finalised cameras. Otherwise, for
    each outgoing edge of the vertex, u, I first check if it’s a toll road. If it is, I don’t not consider that
    edge. Otherwise, if vertex u has not been discovered or finalised, I add it to the min heap and up heap it, and
    keeping track of the vertex v, so we can recover the path. Else if the current distance to vertex u is greater than
    the distance from vertex v plus the edge weight connecting v and u, then I update the value of edge distance to be
    that, and also the previous vertex to be equal to v, and upheap that node because updating the node may have
    violated the heap property. Because we can find the node in the min-heap in O(1) time the upheap has O(logV) time
    complexity. Then I move the vertex v to finalised. Whether or not vertex v is finalised or not, I record where each
    vertex is located in the finalised list in a list called position in finalised list of size V so that I can access
    the previous vertex when recovering the shortest path in O(1) time.
    :param source: source vertex
    :param ad_list: adjacency list
    :param camera_list: list of whether each vertex is a camera or list
    :return: finalised camera list, finalised list, position of each vertex in the finalised list
    :time complexity: O(VlogV + ElogV) time complexity where V is the number of vertices and E is the number of edges.
    :space complexity: O(V) space complexity where V is the number of vertices
    """
    vertices = [-2 for _ in range(6105)]
    finalised = []
    min_heap = [None, [source, 0, False, source]]
    vertices[source] = 1
    shortest_path_camera_list = []
    pos_in_finalised_list = [-1 for _ in range(6105)]
    while len(min_heap) >= 2:
        vertex = get_min(min_heap, vertices)
        if not vertex[2]:
            vertex_num = vertex[0]
            vertex_dist = vertex[1]
            vertex_adj_list = ad_list[vertex_num]
            for a in range(len(vertex_adj_list)):
                adj_vertex = vertex_adj_list[a]
                if not adj_vertex[2]:
                    adj_vertex_num = adj_vertex[0]
                    edge_dist = adj_vertex[1]
                    adj_vertex_dist = vertex_dist + edge_dist
                    if vertices[adj_vertex_num] == -2:
                        red_light = camera_list[adj_vertex_num]
                        vertex_info = [adj_vertex_num, adj_vertex_dist, red_light, vertex_num]
                        min_heap.append(vertex_info)
                        last_node_loc = len(min_heap) - 1
                        vertices[adj_vertex_num] = last_node_loc
                        up_heap(min_heap, last_node_loc, vertices)
                    elif vertices[adj_vertex_num] != -1 and min_heap[vertices[adj_vertex_num]][1] > adj_vertex_dist:
                        min_heap_loc = vertices[adj_vertex_num]
                        min_heap[min_heap_loc][1] = adj_vertex_dist
                        min_heap[min_heap_loc][3] = vertex_num
                        up_heap(min_heap, min_heap_loc, vertices)
        finalised_info = [vertex[0], vertex[1], vertex[3]]
        finalised.append(finalised_info)
        last_element_index = len(finalised) - 1
        pos_in_finalised_list[vertex[0]] = last_element_index
        if vertex[2]:
            camera_info = [vertex[0], vertex[1], vertex[3]]
            shortest_path_camera_list.append(camera_info)
    results = (shortest_path_camera_list, finalised, pos_in_finalised_list)
    return results


def get_min(my_heap, vertex_loc):
    """
    This function gets the minimum of the heap, swaps the root with the last node and sinks the root node. It also
    updates the position indicator of vertices in the heap.
    :param my_heap: a min heap
    :param vertex_loc: a list to keep track of where each vertex is located in the min heap
    :return: the root node of the min heap (vertex with the smallest distance)
    :time complexity: O(logV) where V is the number of vertices
    :space complexity: O(1) constant space
    """
    min_vertex = my_heap[1]
    last_vertex_num = my_heap[len(my_heap)-1][0]
    swap(my_heap, 1, len(my_heap) - 1)
    vertex_loc[last_vertex_num] = 1
    vertex_loc[min_vertex[0]] = -1
    my_heap.pop()
    sift_down(my_heap, 1, vertex_loc)
    return min_vertex


def swap(array, i, j):
    """
    This function swaps elements from two given indices in an array
    :param array: an array
    :param i: index 1
    :param j: index 1
    :return: none
    :time complexity: O(1) constant time
    :space complexity: O(1) constant space
    """
    array[i], array[j] = array[j], array[i]


def sift_down(heap, node, vertices_location):
    """
    This function sinks a node until the heap property is satisfied. It also updates the position indicator of vertices
    in the heap.
    :param heap: a min heap
    :param node: node
    :param vertices_location: a list to keep track of where each vertex is located in the min heap
    :return: none
    :time complexity: O(logV) time complexity where V is the number of vertices
    :space complexity: O(1) constant space
    """
    parent = node
    heap_size = len(heap) - 1
    while 2*parent <= heap_size:
        smallest_child = find_smallest_child(heap, parent)
        if heap[parent][1] < heap[smallest_child][1]:
            return
        else:
            swap(heap, parent, smallest_child)
            index_a = heap[parent][0]
            index_b = heap[smallest_child][0]
            swap(vertices_location, index_a, index_b)
            parent = smallest_child


def find_smallest_child(a_heap, k):
    """
    This function finds the smallest child of a given parent.
    :param a_heap: a min heap
    :param k: the parent
    :return: smallest child
    :time complexity: O(1) constant time
    :space complexity: O(1) constant space
    """
    if 2*k == (len(a_heap) - 1) or a_heap[2*k][1] < a_heap[2*k + 1][1]:
        return 2*k
    else:
        return 2*k + 1


def up_heap(heap, current_node, vertex_loc):
    """
    This function rises a node until the heap property is satisfied. It also updates the position indicator of vertices
    in the heap.
    :param heap: a min heap
    :param current_node: the node we are rising
    :param vertex_loc: a list to keep track of where each vertex is located in the min heap
    :return: none
    :time complexity: O(logV) time complexity where V is the number of vertices
    :space complexity: O(1) constant space
    """
    while current_node > 1 and heap[current_node][1] < heap[current_node//2][1]:
        swap(heap, current_node, current_node//2)
        index_f = heap[current_node][0]
        index_g = heap[current_node//2][0]
        swap(vertex_loc, index_f, index_g)
        current_node //= 2


def main_fn():
    """
    To recover the shortest path to k closest cameras, for each camera up until k or the size of the finalised camera
    list, because we record the previous vertex to that node that is the shortest path, and we know exactly, that is
    O(1) time, where each vertex is located in the finalised list if its finalised given by the finalised position
    list, we add the previous vertex and then the previous vertex of the previous vertex’s location in the finalised
    list is given by the finalised position list. If the source vertex is red light camera, then we do not find the
    shortest path.
    :param: none
    :return: none
    :time complexity: O(VlogV + ElogV + P) time complexity where V is the number of vertices, E is the number of edges
    and P is the total number of edges printed in the output
    :space complexity: O(V + E + P) space complexity where is the number of vertices, E is the number of edges and P is
    the total number of edges printed in the output
    """
    adjacency_list = read_in_edges("edges.txt")
    red_light_camera_list = read_in_red_light_camera("vertices.txt")
    source_vertex = int(input("Enter your location: "))
    k_val = int(input("Enter k: "))
    if red_light_camera_list[source_vertex]:
        print("Oops! Cannot help, Alice!!! Smile for the camera!")
    else:
        result_lists = find_shortest_path(source_vertex, adjacency_list, red_light_camera_list)
        closest_cameras = result_lists[0]
        if len(closest_cameras) == 0:
            print("Oops! You're stuck here Alice!")
        else:
            finalised_list = result_lists[1]
            positions_in_finalised = result_lists[2]
            if k_val > len(closest_cameras):
                camera_number = len(closest_cameras)
            else:
                camera_number = k_val
            for x in range(camera_number):
                camera = closest_cameras[x][0]
                distance_to_camera = closest_cameras[x][1]
                shortest_path = [camera]
                previous_vertex = closest_cameras[x][2]
                while previous_vertex != source_vertex:
                    shortest_path.append(previous_vertex)
                    previous_vertex_final_pos = positions_in_finalised[previous_vertex]
                    previous_vertex = finalised_list[previous_vertex_final_pos][2]
                shortest_path.reverse()
                shortest_path_string = "{}".format(source_vertex)
                for vertex in shortest_path:
                    shortest_path_string += " --> {}".format(vertex)
                print("\nCamera {} : {}  Distance from your location: {}".format(x+1, camera, distance_to_camera))
                print("Shortest path: {}".format(shortest_path_string))


if __name__ == '__main__':
    main_fn()
