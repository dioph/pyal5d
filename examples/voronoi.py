import cv2
import numpy as np

from pyal5d.path_planning import *

# Red
RED_LOW_THRESH = [(0, 100, 100), (10, 255, 255)]
RED_HIGH_THRESH = [(170, 100, 100), (180, 255, 255)]

if __name__ == '__main__':

    # 'big_sample.png'
    # scene = cv2.imread('big_sample.png')
    # REAL_ORIGIN = np.array([14, 230])
    # REAL_GOAL = np.array([500, 240])

    # 'short_sample.png'
    scene = cv2.imread('short_sample.png')
    REAL_ORIGIN = np.array([92, 11])
    REAL_GOAL = np.array([121, 69])

    show_then_clear(scene, 'Original scene')

    scene = cv2.GaussianBlur(scene, (3, 3), sigmaX=0)

    scene = threshold_img(scene, RED_LOW_THRESH, RED_HIGH_THRESH)
    height, width = scene.shape

    mask = np.ones((3, 3), )
    scene = cv2.morphologyEx(scene, cv2.MORPH_OPEN, mask)
    scene = cv2.morphologyEx(scene, cv2.MORPH_CLOSE, mask)

    for i in range(height):
        for j in [0, width - 1]:
            cv2.floodFill(scene, None, (j, i), 0)

    LABEL = 1

    count_img = scene.copy()
    for i in range(height):
        for j in range(width):
            if count_img[i, j] == 255:
                cv2.floodFill(count_img, None, (j, i), LABEL)
                LABEL += 1

    REGIONS = LABEL - 1
    LABEL = 1
    INC = 255 // REGIONS

    for i in range(height):
        for j in range(width):
            if scene[i, j] == 255:
                cv2.floodFill(scene, None, (j, i), LABEL)
                LABEL += INC

    obstacles = np.atleast_2d(np.where(scene > 0)).T

    voronoi = get_voronoi(scene, obstacles)

    v_img = voronoi.astype(np.uint8)
    show_then_clear(v_img, "Voronoi\'s diagram")

    img = cv2.Canny(v_img, 50, 150)
    img = cv2.dilate(img, np.ones((3, 3)))
    show_then_clear(img)

    img = cv2.erode(img, np.ones((2, 2)))
    show_then_clear(img)

    img = cv2.erode(img, np.ones((2, 2)))
    show_then_clear(img)

    img[img > 0] = 255

    paths = np.atleast_2d(np.where(img == 255))
    paths = paths.T

    img[REAL_ORIGIN[0], REAL_ORIGIN[1]] = 120
    img[REAL_GOAL[0], REAL_GOAL[1]] = 120

    show_then_clear(img, "Shown origin and goal")

    ORIGIN_INDEX = np.linalg.norm(np.abs(REAL_ORIGIN - paths), axis=1).argmin()
    GOAL_INDEX = np.linalg.norm(np.abs(REAL_GOAL - paths), axis=1).argmin()

    ORIGIN = paths[ORIGIN_INDEX]
    GOAL = paths[GOAL_INDEX]

    adj_list = create_adjacency_list(paths)
    adj_list = create_adjacency_list_neighborhood(adj_list, img)
    adj_list[ORIGIN_INDEX].set_distance(0)

    assert adj_list[ORIGIN_INDEX].get_distance() == 0

    linked_nodes_array = dijkstra(GOAL_INDEX, adj_list)
    trajectory = reconstruct_path(ORIGIN, GOAL, linked_nodes_array)

    color_img = np.dstack([img, img, img])

    for step in trajectory:
        i, j = step
        color_img[i, j, :] = np.asarray([0, 0, 255])

    show_then_clear(color_img, "Planned path")
