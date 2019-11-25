import cv2
import numpy as np

__all__ = [
    'threshold_img',
    'show_then_clear',
    'get_voronoi',
    'create_adjacency_list',
    'create_adjacency_list_neighborhood',
    'dijkstra',
    'reconstruct_path',
]


class Node:
    def __init__(self, position=None, distance=np.inf, previous=None):
        self.position = position
        self.distance = distance
        self.previous = previous
        self.neighborhood = np.empty((0,), dtype=Node)

    def set_neighborhood(self, method, *args, **kwargs):
        self.neighborhood = method(*args, **kwargs)

    def set_distance(self, dist):
        self.distance = dist

    def set_previous(self, previous):
        self.previous = previous

    def get_position(self):
        return self.position

    def get_previous(self):
        return self.previous

    def get_neighborhood(self):
        return self.neighborhood

    def get_distance(self):
        return self.distance


def threshold_img(image, low_thresh, high_thresh):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lowimg = cv2.inRange(hsv, *low_thresh)
    highimg = cv2.inRange(hsv, *high_thresh)
    return lowimg + highimg


def show_then_clear(image, window=''):
    cv2.imshow(window, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def get_voronoi(image, obstacles):
    voronoi = image.copy()
    w, h = image.shape[::-1]
    for i in range(w):
        for j in range(h):
            element = np.linalg.norm(np.abs(np.asarray([i, j]) - obstacles), axis=1).argmin()
            x, y = obstacles[element]
            voronoi[i, j] = image[x, y]
    return voronoi


def create_adjacency_list(paths):
    length = paths.shape[0]
    adjacency_list = np.zeros((length,), dtype=Node)
    for i in range(length):
        adjacency_list[i] = Node(position=paths[i])
    return adjacency_list


def create_adjacency_list_neighborhood(adjacency_list, laplacian_img):
    length = np.size(adjacency_list)
    for i in range(length):
        position = adjacency_list[i].get_position()
        adjacency_list[i].set_neighborhood(
            method=neighborhood_from_laplacian_img,
            position=position,
            image=laplacian_img,
            adjacency_list=adjacency_list
        )
    return adjacency_list


def neighborhood_from_laplacian_img(position, image, adjacency_list):
    neighborhood = np.empty((0,), dtype=Node)
    i, j = position
    neighborhood_ij = np.array([
        [a, b] for a in range(i - 1, i + 2)
        for b in range(j - 1, j + 2)
        if (0 <= a < image.shape[0])
           and (0 <= b < image.shape[1])
           and image[a, b] == 255
    ])
    for neighbor_ij in neighborhood_ij:
        for node in adjacency_list:
            if np.array_equal(neighbor_ij, node.get_position()):
                neighborhood = np.append(neighborhood, node)
    return neighborhood


def dijkstra(goal, adjacency_list):
    linked_nodes = np.empty((0,), dtype=Node)
    while np.size(adjacency_list):
        u = minor_distance(adjacency_list)
        element = adjacency_list[u]
        linked_nodes = np.append(linked_nodes, element)
        adjacency_list = np.delete(adjacency_list, u, axis=0)
        if np.array_equal(element.get_position(), goal):
            break
        for neighbor in element.get_neighborhood():
            alt = element.get_distance() + np.linalg.norm(element.get_position() - neighbor.get_position())
            if alt < neighbor.get_distance():
                neighbor.set_distance(alt)
                neighbor.set_previous(element)
    return linked_nodes


def minor_distance(adjacency_list):
    dists = np.asarray([node.get_distance() for node in adjacency_list])
    return dists.argmin()


def reconstruct_path(origin, goal, linked_nodes):
    path = np.empty((0,), dtype=Node)
    gi = goal_index_in_adj_list(goal, linked_nodes)
    oi = origin_index_in_adj_list(origin, linked_nodes)
    u = linked_nodes[gi]
    source = linked_nodes[oi]
    if u.get_previous() is not None or \
            np.array_equal(gi.get_position(), source.get_position()):
        while u is not None:
            path = np.insert(path, 0, u)
            u = u.get_previous()
    return np.asarray([p.get_position() for p in path])


def goal_index_in_adj_list(goal, adjacency_list):
    positions = np.asarray([node.get_position() for node in adjacency_list])
    return np.linalg.norm(goal - positions, axis=1).argmin()


def origin_index_in_adj_list(origin, adjacency_list):
    positions = np.asarray([node.get_position() for node in adjacency_list])
    return np.linalg.norm(origin - positions, axis=1).argmin()
