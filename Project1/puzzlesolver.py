import numpy as np
import collections
import os

class node:
    def __init__(self, nodekey, node, parent, parentkey, move, cost):
        self.nodekey = nodekey
        self.node = node
        self.parent = parent
        self.parentkey = parentkey
        self.move = move
        self.cost = cost

def start():
    #Entering the intial state
    print("Input the starting state")
    inputnode = []
    for i in range(9):
        a = int(input())
        while((a<0 or a>8) or a in inputnode):
            print("Invalid Number")
            a = int(input())
        inputnode.append(a) 
    return np.reshape(inputnode,(3, 3)) 

def BlankTile(node):
    i,j=np.where(node==0)[0],np.where(node==0)[1]
    return i,j

def ActionMoveUp(node):
    mover = node.copy()
    i,j = BlankTile(node)
    if i-1<0:
        return False      
    else:
        mover[i,j] = mover[i-1,j]
        mover[i-1,j] = 0
    return mover

def ActionMoveDown(node):
    mover = node.copy()
    i,j = BlankTile(node)
    if i+1>2:
        return False    
    else:
        mover[i,j] = mover[i+1,j]
        mover[i+1,j] = 0
    return mover 

def ActionMoveLeft(node):
    mover = node.copy()
    i,j = BlankTile(node)
    if j-1<0:
        return False
    else:
        mover[i,j] = mover[i,j-1]
        mover[i,j-1] = 0
    return mover 

def ActionMoveRight(node):
    mover = node.copy()
    i,j = BlankTile(node)
    if j+1>2:
        return False 
    else:
        mover[i,j] = mover[i,j+1]
        mover[i,j+1] = 0
    return mover

def Explore(move, node_state):

    if move == 1:
        return ActionMoveLeft(node_state)
    
    elif move == 2:
        return ActionMoveUp(node_state)
    
    elif move == 3:
        return ActionMoveRight(node_state)
    
    elif move == 4:
        return ActionMoveDown(node_state)

    else:
        return None 

def addkey(node_state):
    array_idx = np.reshape(node_state,9)
    string_idx = int(''.join(str(i) for i in array_idx))
    return string_idx


def BFS(initial_state, goal):
    
    actions = [1,2,3,4]
    visited = []    
    visited.append(initial_state)
    startqueue = collections.deque([initial_state]) 
    nextnode = {addkey(startqueue[0].node) : startqueue[0].nodekey}
    nodekey = 0

    while startqueue:
        current = startqueue.popleft()
        if current.node.tolist() == goal.tolist():
            print('Goal State Reached!!!')
            return current, visited

        for action in actions:
            child = Explore(action, current.node)
            if child is not False:
                nodekey +=1
                child = node(nodekey, np.array(child),current,nodekey-1,action,0)
                
                if addkey(child.node) not in nextnode:
                #if any(x.node_state.tolist() == child_node.node_state.tolist() for x in node_visited) == False :
                    startqueue.append(child)
                    nextnode[addkey(child.node)] = child.nodekey
                    visited.append(child)

                    if child.node.tolist() == goal.tolist():
                        print('Goal State Reached!!!')
                        return child, visited
    
    return None, None

def backtrack(goal):  
    shortest = []  
    shortest.append(goal)
    parentnode = goal.parent
    while parentnode is not None:
        shortest.append(parentnode)
        temp = parentnode.parent
        parentnode = temp
    return list(reversed(shortest))

# Text file edit for path
def path(path_formed):
    if os.path.exists("nodePath.txt"):   
        os.remove("nodePath.txt")

    f = open("nodePath.txt", "a")
    for node in path_formed:   
        if node.parent is not None:
            f.write(str(node.nodekey) + "\t" + str(node.parent.nodekey) + "\t" + str(node.cost) + "\t" + str(node.node) + "\n")
        if node.parent is None:
            f.write(str(node.nodekey) + "\t" + str(node.cost) + "\t" + str(node.node) + "\n")
    f.close()

def allnodes(visited_nodes):
    if os.path.exists("Nodes.txt"):     
        os.remove("Nodes.txt")

    f = open("Nodes.txt", "a")
    for node in visited_nodes:
        f.write('[')
        for i in range(0,len(node.node)):
            for j in range(len(node.node)):
                f.write(str(node.node[j][i]) + " ")
        f.write(']')
        f.write("\n")
    f.close()

def nodeinfo(visited):
    if os.path.exists("NodesInfo.txt"):     
        os.remove("NodesInfo.txt")

    f = open("NodesInfo.txt", "a")
    for n in visited:     
        if n.parent is not None:
            f.write(str(n.nodekey) + "\t" + str(n.parent.nodekey) + "\t" + str(n.cost) + "\n")
    f.close()


def main():
    input = start()
    input = node(0, input.T, None, None, 0, 0)
    goal = np.array([[1,2,3],[4,5,6],[7,8,0]])
    goal, visited = BFS(input, goal.T)
    x = backtrack(goal)
    path(x)
    allnodes(visited)
    nodeinfo(visited)

if __name__ == "__main__":
    main()