''' A* for ROS implementation'''
import numpy as np
import time
import math
import matplotlib
import matplotlib.pyplot as plt
from collections import deque
import matplotlib.patches as patches
matplotlib.use("TkAgg")

plt.ion()

width = 10

height = 10
th = 15


def obstaclecheck_circle(x, y, r, c):
    tot = r + c

    circle1 = ((np.square(x - 2)) + (np.square(y - 2)) <= np.square(1 + tot))
    circle2 = ((np.square(x - 2)) + (np.square(y - 8)) <= np.square(1 + tot))

    if circle1 or circle2:
        return True
    else:
        return False


def obstaclecheck_rectangle(r, c, x, y):
    tot = r + c

    rect1 = (x >= 3.75 - tot) and (x <= 6.25 + tot) and (y >= 4.25 - tot) and (y <= 5.75 + tot)
    rect2 = (x >= 7.25 - tot) and (x <= 8.75 + tot) and (y >= 2 - tot) and (y <= 4 + tot)

    if rect1 or rect2:
        return True
    else:
        return False


def obstaclecheck_square(x, y, r, c):
    tot = r + c
    square1 = (x >= 0.25 - tot) and (x <= 1.75 + tot) and (y >= 4.25 - tot) and (y <= 5.75 + tot)

    if square1:
        return True
    else:
        return False


def is_valid(x, y, r, c):
    if ((x - r - c <= 0) or (y - r - c <= 0) or (x + r + c >= width) or (+ r + c >= height)):
        return False
    elif obstaclecheck_circle(x, y, r, c):
        return False
    elif obstaclecheck_square(x, y, r, c):
        return False
    elif obstaclecheck_rectangle(r, c, x, y):
        return False
    else:
        return True


def is_goal(current, goal):
    dt = math.dist((current[0], current[1]), (goal[0], goal[1]))

    if dt < 0.25:
        return True
    else:
        return False


def threshold(x, y, th, theta):
    x = (round(x * 10) / 10)
    y = (round(y * 10) / 10)
    th = (round(th / theta) * theta)
    return (x, y, th)


def action_model(rpm1, rpm2):
    actions = [[rpm1, 0], [0, rpm1], [rpm1, rpm1], [0, rpm2], [rpm2, 0], [rpm2, rpm2], [rpm1, rpm2], [rpm2, rpm1]]
    return actions


def cost(Xi, Yi, Thetai, UL, UR,plot = True):
    Thetai = Thetai % 360
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    Xn = Xi
    Yn = Yi
    Thetan = 3.14 * Thetai / 180

    D = 0
    points = []
    while t < 1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5 * r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5 * r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        D = D + math.sqrt(math.pow((0.5 * r * (UL + UR) * math.cos(Thetan) * dt), 2) + math.pow(
            (0.5 * r * (UL + UR) * math.sin(Thetan) * dt), 2))
        # D = round(D * 8) / 8
        points.append([(Xs, Xn), (Ys, Yn)])
    Thetan = 180 * (Thetan) / 3.14
    tup_return = (*threshold(Xn, Yn, Thetan, th), D, UL, UR, points)
    return tup_return

def threshold(x, y, th, theta):
    x = (round(x * 10) / 10)
    y = (round(y * 10) / 10)
    th = (round(th / theta) * theta)
    return (x, y, th)

# def valid_neighbors(current_node,rad, clearance, L_rpm, R_rpm):
#     coords = []
#     for action in action_model(L_rpm,R_rpm):
#         x, y, theta,cost_,UL,UR, points = cost(*current_node,*action)
#         if (x, y) not in coords and is_valid(x, y, rad,clearance):
#             coords.append((x, y))
#             for points_ in points:
#                 plt.plot(*points_, color="red", alpha=0.2)
#                 plt.pause(0.0000000000000000000000000000000000000000000000000000000000000000000000001)
#             yield x, y, theta,cost_,UL,UR


def valid_neighbors(current_node,rad, clearance, L_rpm, R_rpm):
    for action in action_model(L_rpm,R_rpm):
        x, y, theta,cost_,UL,UR, points = cost(*current_node,*action)
        if is_valid(x, y, rad,clearance):
            plt.plot([current_node[0], x], [current_node[1], y], color="red", alpha=0.2)
            plt.pause(0.0000000000000000000000000000000000000000000000000000000000000000000000001)
            yield x, y, theta,cost_,UL,UR

def plot_map():
    figure, axes = plt.subplots()
    axes.set(xlim=(0, 10), ylim=(0, 10))

    circle_1 = plt.Circle((2, 2), 1, fill='True')
    circle_2 = plt.Circle((2, 8), 1, fill='True')

    rect1 = patches.Rectangle((0.25, 4.25), 1.5, 1.5, color='green')
    rect2 = patches.Rectangle((3.75, 4.25), 2.5, 1.5, color='red')
    rect3 = patches.Rectangle((7.25, 2), 1, 2, color='yellow')

    axes.set_aspect('equal')
    axes.add_artist(circle_1)
    axes.add_artist(circle_2)
    axes.add_patch(rect1)
    axes.add_patch(rect2)
    axes.add_patch(rect3)
    plt.show()

def plot_result(path):

    start_node = path[0]
    goal_node = path[-1]
    plt.plot(start_node [0], start_node[1], "Dw")
    plt.plot(goal_node[0], goal_node[1], "Dg")

    l = 0
    for i,(x,y,theta) in enumerate(path[:-1]):
        n_x, n_y,theta = path[i+1]
        plt.plot([x, n_x], [y, n_y], color="blue")

    plt.show()
    plt.pause(30)
    plt.close('all')



def A_star(start_node,goal_node, rad, clearance, L_rpm, R_rpm):
    paths = {}
    open_list = deque()
    visited_nodes = {}
    open_list.append((start_node,float('inf'),0))
    while open_list:
        current_node, dist, cost_to_come = open_list.popleft()
        visited_nodes[(current_node[0],current_node[1])] = 1
        if dist <= 0.5:#Distance to be traveled is less than or equal to threshold
            goal_node = current_node
            break
        neighbors = set(valid_neighbors(current_node,rad, clearance, L_rpm, R_rpm))
        for n_x, n_y, n_theta, n_cost,UL,UR in neighbors:
            dist = math.dist((n_x, n_y),goal_node[:2])
            if visited_nodes.get((n_x, n_y)) == 1:
                continue
            new_cost = cost_to_come + n_cost
            for i,item in enumerate(open_list):
                if item[1] + item[2] > new_cost + dist:
                    open_list.insert(i,((n_x, n_y, n_theta), dist, new_cost))
                    break
            else:
                open_list.append(((n_x, n_y, n_theta), dist, new_cost))
            paths[(n_x, n_y, n_theta)] = current_node
    current_node = goal_node
    path = [goal_node]
    while current_node != start_node:
        current_node = paths[current_node]
        path.append(current_node)
    return path[::-1]




r = float(input('Please enter the radius of the robot: '))
c = float(input('Please enter the clearance value: '))
L_rpm = int(input('Please enter the left angular velocity: '))
R_rpm = int(input('Please enter the right angular velocity: '))


a1,b1,c1 = input("Enter the co-ordinates for the start node: ").split(",",3)
start_node = tuple([int(a1), int(b1), int(c1)])
while not is_valid(start_node[0],start_node[1],r,c):
    print('This is in the obstacle space: ')
    a1, b1, c1 = input("Enter the co-ordinates for the start node: ").split(",", 3)
    start_node = tuple([int(a1), int(b1), int(c1)])

a2,b2,c2 = input("Enter the co-ordinates for the goal node: ").split(",",3)
goal_node = tuple([int(a2),int(b2),int(c2)])
while not is_valid(goal_node[0],goal_node[1],r,c):
    print('This is in the obstacle space: ')
    a2, b2, c2 = input("Enter the co-ordinates for the goal node: ").split(",", 3)
    goal_node = tuple([int(a2), int(b2), int(c2)])

plot_map()
result = A_star(start_node,goal_node, r, c, L_rpm, R_rpm)
plot_result(result)
