# Program to load obstacle course for Lab 4 - RRT

# usage:  python rrt.py obstacles_file start_goal_file


from __future__ import division
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import matplotlib.lines as mlines
import numpy as np
import random, math
from RRT import *

vertices = []

def build_obstacle_course(obstacle_path, ax):
    global vertices
    vertices = list()
    codes = [Path.MOVETO]
    with open(obstacle_path) as f:
        quantity = int(f.readline())
        lines = 0
        for line in f:
            coordinates = tuple(map(int, line.strip().split(' ')))
            if len(coordinates) == 1:
                codes += [Path.MOVETO] + [Path.LINETO]*(coordinates[0]-1) + [Path.CLOSEPOLY]
                vertices.append((0,0)) #Always ignored by closepoly command
            else:
                vertices.append(coordinates)
    vertices.append((0,0))
    vertices = np.array(vertices, float)
    path = Path(vertices, codes)
    pathpatch = patches.PathPatch(path, facecolor='xkcd:blue', edgecolor='xkcd:blue')

    ax.add_patch(pathpatch)
    ax.set_title('Rapidly-exploring Random Tree')

    ax.dataLim.update_from_data_xy(vertices)
    ax.autoscale_view()
    ax.invert_yaxis()

    return path

def add_start_and_goal(start_goal_path, ax):
    start, goal = None, None
    with open(start_goal_path) as f:
        start = tuple(map(int, f.readline().strip().split(' ')))
        goal  = tuple(map(int, f.readline().strip().split(' ')))

    ax.add_patch(patches.Circle(start, facecolor='xkcd:red'))
    ax.add_patch(patches.Circle(goal, facecolor='xkcd:green'))

    # plt.plot([start[0], goal[0]], [start[1], goal[1]], marker = 'o', color = 'xkcd:red')

    return start, goal

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('obstacle_path',
                        help="File path for obstacle set")
    parser.add_argument('start_goal_path',
                        help="File path for obstacle set")
    parser.add_argument('step_size',
                        help="Step size for each iteration")
    args = parser.parse_args()

    fig, ax = plt.subplots()
    path = build_obstacle_course(args.obstacle_path, ax)
    start, goal = add_start_and_goal(args.start_goal_path, ax)
    step = float(args.step_size)

    ax.add_patch(patches.Circle(start, radius = 10, facecolor='xkcd:red'))
    ax.add_patch(patches.Circle(goal, radius = 10, facecolor='xkcd:green'))

    rrt = RRT(vertices, start, goal, step)
    rrt.grow_tree()

    


    plt.show()