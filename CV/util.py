import numpy as np
import cv2


def extrapolate_points(size, corners):
    w, h = size
    get = lambda x, y: corners[y * h + x, 0]

    def extend(edge):
        avg = np.average(np.diff(edge, axis=0), axis=0)
        return edge[0] - avg, edge[len(edge) - 1] + avg

    def calculate_hline(y):
        return extend(corners[y * h : y * h + w, 0])

    def calculate_vline(x):
        return extend(corners[x : h * h + x : h, 0])

    L, R, U, D = range(4)
    BWD, FWD = range(2)

    def calculate_corners(edges):
        extended = [extend(i) for i in edges]
        return np.float32(
            [
                np.average([extended[L][BWD], extended[U][BWD]], axis=0),
                np.average([extended[R][BWD], extended[U][FWD]], axis=0),
                np.average([extended[L][FWD], extended[D][BWD]], axis=0),
                np.average([extended[R][FWD], extended[D][FWD]], axis=0),
            ]
        )

    full_points = [[None for _ in range(9)] for _ in range(9)]
    for x in range(w):
        for y in range(h):
            full_points[x + 1][y + 1] = get(x, y)

    edges = [[], [], [], []]
    for y in range(h):
        left, right = calculate_hline(y)
        edges[L].append(left)
        edges[R].append(right)
        full_points[0][y + 1] = left
        full_points[8][y + 1] = right

    for x in range(h):
        top, bottom = calculate_vline(x)
        edges[U].append(top)
        edges[D].append(bottom)
        full_points[x + 1][0] = top
        full_points[x + 1][8] = bottom

    outer_corners = calculate_corners(edges)
    full_points[0][0] = outer_corners[0]
    full_points[8][0] = outer_corners[1]
    full_points[0][8] = outer_corners[2]
    full_points[8][8] = outer_corners[3]

    return full_points


def draw_points(img, points):
    new = img.copy()
    for col in points:
        for point in col:
            cv2.circle(new, (int(point[0]), int(point[1])), 5, (0, 255, 0))
    return new
