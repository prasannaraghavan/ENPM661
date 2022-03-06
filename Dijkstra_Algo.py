import numpy as np
import cv2
import math
from queue import PriorityQueue
import matplotlib.pyplot as plt

map = np.zeros((250, 400), np.uint8)
map_x = map.shape[0]
map_y = map.shape[1]
obstacle_map = []
for y in range(250):
    for x in range(400):
        #Circle 
        if (185-y)**2+(300-x)**2 <= (40**2):
            map[250-y,x] = 1
            obstacle_map.append([x,y])
        #Hexagon
        h1 = y - 0.577*x - 24.97 
        h2 = y + 0.577*x - 255.82
        h3 = x - 235 
        h6 = x - 165 
        h5 = y + 0.577*x - 175 
        h4 = y - 0.577*x + 55.82 
        if(h1<0 and h2<0 and h3<0 and h4>0 and h5>0 and h6>0):
            map[250-y,x]=1
            obstacle_map.append([x,y])
        #Polygon Obstacle
        l1 = y - ((0.316) *x) - 173.608  
        l2 = y + (1.23 * x) - 229.34 
        l3 = y + (3.2 * x) - 436 
        l4 = y - 0.857*x - 111.42          
        l5 = y + (0.1136*x) - 189.09
        if (l1<0 and l5>0 and l4>0):
            map[250-y,x]=1 
            obstacle_map.append([x,y])
        if (l2>0 and l5<0 and l3<0):
            map[250-y,x]=1 
            obstacle_map.append([x,y])

#plt.imshow(map)
#plt.show()

def ValidMove(move):
    if (move[0] < map_y and move[0] > 0 and move[1] < map_x and move[1] > 0):
        if map[move[1]][move[0]] == 0:
            return True
    else:
        return False
    
stx , sty = [int(x) for x in input("Enter start node values separated by a space: ").split()]
while ValidMove([stx,sty]) == False:
    print("You have entered an invalid start node \nEnter valid coordinates: ")
    stx , sty = [int(x) for x in input("Enter two values: ").split()]
print("The starting coordinates are: [",stx,",",sty,"]")
glx , gly = [int(x) for x in input("Enter goal node values separated by a space: ").split()]
while ValidMove([glx,gly]) == False:
    print("You have entered an invalid Goal node \nEnter valid coordinates: ")
    glx , gly = [int(x) for x in input("Enter two values: ").split()]
while [stx,sty]==[glx,gly]:
    print("The start and the goal cannot be the same \nEnter valid coordinates: ")
    glx , gly = [int(x) for x in input("Enter two values: ").split()]
print("The goal coordinates are: [",glx,",",gly,"]")


start_node = [stx,sty]
goal_node = [glx,gly]


class Node:
    def __init__(self, inx, cost, parent):
        self.inx = inx
        self.x = inx[0]
        self.y = inx[1]
        self.cost = cost
        self.parent = parent


open_node = PriorityQueue()
visited_nodes = set([])
node_objects = {}
visit = {}

def explore(current_node):
    valid_moves = []
    all_moves = [(0,1),(1,0),(-1,0),(0,-1),(1,1),(-1,11),(-1,1),(1,-1)]
    for i in range(len(all_moves)):
        nx = current_node.x+all_moves[i][0]
        ny = current_node.y+all_moves[i][1]
        if ValidMove((nx,ny)):
            cost = math.sqrt((all_moves[i][0])**2+(all_moves[i][1])**2)
            valid_moves.append([(nx,ny),cost]) 
    return valid_moves


cost2come = {}
for i in range(0, map_y):
    for j in range(0, map_x):
        cost2come[str([i, j])] = np.Infinity

cost2come[str(start_node)] = 0 
visited_nodes.add(str(start_node))
node = Node(start_node, 0, None)
node_objects[str(node.inx)] = node
open_node.put([node.cost, node.inx])
plot_map = map
img_show = np.dstack([map.copy()*255, map.copy()*255, map.copy()*255])
 
frameSize = (map_y, map_x)

out = cv2.VideoWriter('Dijkstra_Video.mp4',cv2.VideoWriter_fourcc(*'DIVX'), 3500, frameSize)

while not open_node.empty():
    node_temp = open_node.get()
    node = node_objects[str(node_temp[1])]
    if node_temp[1][0] == goal_node[0] and node_temp[1][1] == goal_node[1]:
        node_objects[str(goal_node)] = Node(goal_node, node_temp[0], node)
        break
    
    for next_node, cost in explore(node):
    
        if str(next_node) in visited_nodes:
            cost_temp = cost + cost2come[str(node.inx)]
            if cost_temp < cost2come[str(next_node)]:
                cost2come[str(next_node)] = cost_temp
                node_objects[str(next_node)].parent = node
        else:
            visited_nodes.add(str(next_node))
            img_show[next_node[1], next_node[0], :] = np.array([0,0,255])
            absolute_cost = cost + cost2come[str(node.inx)]
            cost2come[str(next_node)] = absolute_cost
            new_node = Node(next_node, absolute_cost, node_objects[str(node.inx)])
            node_objects[str(next_node)] = new_node
            open_node.put([absolute_cost, new_node.inx])
            vid_show = img_show.astype(np.uint8)
            out.write(vid_show)
            #cv2.imshow("Finding",img_show)
            #cv2.waitKey(1)      
goal_path = node_objects[str(goal_node)]
parent_node = goal_path.parent 
final_path = []
while parent_node:
    img_show[parent_node.inx[1], parent_node.inx[0],:] = np.array([255,0,0])
    #map[parent_node.inx[1], parent_node.inx[0]] = 1
    final_path.append([parent_node.inx[1], parent_node.inx[0]])
    parent_node = parent_node.parent
    vid_show = img_show.astype(np.uint8)
    out.write(vid_show)
out.release()
