import numpy as np
import pathlib
from datetime import datetime

PUBLICATION_NAME = "multi-mpc-2023"
PROJECT_FOLDER = str(pathlib.Path.home()) + "/Documents/publications/" + PUBLICATION_NAME + "/results"

SIMULATIONS_IN_TABLE = ["random-4_straight",
                        "random-8_straight",
                        "random-12_straight",
                        "random-16_straight",
                        # "corridor-16_random",
                        "crossing-16_random-sidewalks",
]
def get_folder_names(project=None):
    if project is None:
        project = PROJECT_FOLDER

    data_folder = project + "/data/"
    metric_folder = project + "/metrics/"
    figure_folder = project + "/figures/"
    return data_folder, metric_folder, figure_folder

def get_latex_folder_name(project=None):
    if project is None:
        project = PROJECT_FOLDER

    latex_table_folder = project + "/../latex/tables/" + get_timestamp() + "/"
    latex_figure_folder = project + "/../latex/figures/"
    return latex_table_folder, latex_figure_folder

def get_timestamp():
    return datetime.now().strftime("%Y_%m_%d")

def get_backup_folder(project=None):
    if project is None:
        project = PROJECT_FOLDER

    return project + "/backup/" + get_timestamp() + "/"


class bcolors:
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    HEADER = BOLD


def print_value(name, value, tab=False):
    if tab:
        print("\t" + bcolors.BOLD + bcolors.UNDERLINE + f"{name}" + bcolors.ENDC + f": {value}")
    else:
        print(bcolors.BOLD + bcolors.UNDERLINE + f"{name}" + bcolors.ENDC + f": {value}")


# MATHS

def get_points_on_line(A, b, obs, r, x_start=100, x_end=-100):

    if abs(A[0]) > 1e-5:
        y_1 = (b - A[1]*x_start) / A[0]
        y_2 = (b - A[1]*x_end) / A[0]
        return [[x_start, y_1], [x_end, y_2]]
    else:
        x_1 = (b - A[0] * 100.) / A[1]
        x_2 = (b + A[0] * 100.) / A[1]
        return [[x_1, 100.], [x_2, -100.]]

def line_intersection(A1, A2, b1, b2, obs, r):
    line1 = get_points_on_line(A1, b1, obs, r)
    line2 = get_points_on_line(A2, b2, obs, r)

    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return [y, x]

def get_points_in_polygon(pos, obs, beta, r_obs, r):

    x_size = 7
    y_size = 3

    A = (obs - pos) / np.linalg.norm(obs - pos)
    b = A.transpose().dot(obs - A * (beta * r))

    coords = []  # Not the nicest way, but easy to program
    intersect_top = line_intersection(A, np.array([0, 1]), b, y_size, obs, r_obs)
    intersect_bot = line_intersection(A, np.array([0, 1]), b, -y_size, obs, r_obs)
    intersect_left = line_intersection(A, np.array([1, 0]), b, 0, obs, r_obs)
    intersect_right = line_intersection(A, np.array([1, 0]), b, x_size, obs, r_obs)

    # print(pos)
    if pos[0] < obs[0] and pos[1] > 0:
        coords.append([0, y_size])  # top left
        if intersect_top[0] < x_size:
            coords.append(intersect_top)
        else:
            coords.append([x_size, y_size])
            coords.append(intersect_right)

        if intersect_bot[0] > 0:
            coords.append(intersect_bot)
            coords.append([0, -y_size])
        else:
            coords.append(intersect_left)
    elif pos[0] > obs[0] and pos[1] > 0:
        coords.append([x_size, y_size])
        if intersect_right[1] > -y_size:
            coords.append(intersect_right)
        else:
            coords.append([x_size, -y_size])
            coords.append(intersect_bot)

        if intersect_left[1] < y_size:
            coords.append(intersect_left)
            coords.append([0, y_size])
        else:
            coords.append(intersect_top)
    else:
        # Run the same thing but in symmetry
        coords_symmetry, _ = get_points_in_polygon(np.array([pos[0], -pos[1]]), np.array([obs[0], -obs[1]]), beta, r_obs, r)
        coords = [np.array([coord[0], -coord[1]]) for coord in coords_symmetry]

    points_on_line = get_points_on_line(A, b, obs, r)
    return coords, points_on_line

