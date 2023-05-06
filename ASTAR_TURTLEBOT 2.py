import cv2
import numpy as np
from queue import PriorityQueue
import time
import matplotlib.pyplot as plt
import math
from copy import deepcopy
import math as m
from random import randint
from scipy.optimize import curve_fit

# start_time = time.time()


from matplotlib.patches import Circle, Rectangle



class ObsMap:
    def __init__(self,robot_radius,obs_clearance):
        self.robot_radius = robot_radius
        self.obs_clearance = obs_clearance
        self.start= 0
        self.end = 600        
        self.fig = plt.figure(figsize=(8,6))
        self.ax = self.fig.subplots()
        self.ax.set_xlim(self.start, self.end)
        self.ax.set_ylim(self.start, 200)

        
        self.ax.grid(which='minor', alpha=0.2)
        self.ax.grid(which='major', alpha=0.5)
        self.ax.set_aspect('equal')
        
        plt.xlabel('x axis')
        plt.ylabel('y axis')
        plt.title("A STAR -RIGID ROBOT MAP")
        # self.offset = (self.end + self.start) / 2

        # self.Map_Clearance_Added()

        self.Original_Map()
        
        
        
    def Original_Map(self):
                                   


        goal_circ = Circle((400,110), radius=50, color='#3C873A')    # Circle on right lower corner of map
        self.ax.add_patch(goal_circ)                            
        rect = Rectangle((150,75),50,125,color='#3C873A') # Square on left side of map
        self.ax.add_patch(rect)
        rect = Rectangle((250,0),50, 125,color='#3C873A') # Square on right side of map
        self.ax.add_patch(rect)

height = 200
width = 600


# # Robot_Radius = 5

threshold = 0.5

# # Test Inputs Starts/Goal


# New Code Implementation ***********************



def euclidean_dist(xi, yi, xf, yf):
    
    e_dist = m.sqrt((xf - xi)**2 + (yf - yi)**2)
    
    return e_dist

def manhattan_dist(xi, yi, xf, yf):
    return abs(xi - xf) + abs(yi - yf)

def diagonal_dist(xi, yi, xf, yf):
    return max( abs(xi - xf), abs(yi - yf)) + (m.sqrt(2)-1) * min( abs(xi - xf), abs(yi - yf))

def robot_traj_coor(x_init,y_init, theta, vL, vR):
    # #print("\n************robot_traj_coor*************")
    
    t_init = 0
    R_tire = 0.38
    length = 3.54
    t_delta = 0.1
    
    xn = x_init
    yn = y_init

    # #print("x_init: ", x_init, "y_init: ", y_init)
    
    theta_rad = 3.14 * theta / 180
    
    curvature = 0

    x_list = [x_init]
    y_list = [y_init]
    
    
    while t_init < 1:
        
        t_init = t_init + t_delta
        
        x_s = xn
        y_s = yn
        
        
        # Three Equations UL, UR, Theta
        xn += (R_tire / 2) * (vL + vR) * m.cos(theta_rad)* t_delta
        yn += (R_tire / 2) * (vL + vR) * m.sin(theta_rad)* t_delta
        theta_rad += (R_tire / length) * (vR - vL) * t_delta
        # x_list.append(xn)
        # y_list.append(yn)

        curvature += euclidean_dist(x_s, y_s, xn, yn)
        
        
        
    theta_deg = 180*(theta_rad) / 3.14
    
    if theta_deg >= 360:
        
        theta_deg = (theta_deg - 360)
        
    if theta_deg <= -360:
        
        theta_deg = (theta_deg + 360)
        
        # curvature influences x and y. curvature = x0 + xi+n 
    
    
    # #print("xn, yn: ", xn, yn)
    # #print("************robot_traj_coor*************\n")
    return xn, yn, theta_deg, curvature, x_list, y_list


def nodes_visited():
    
    already_visited = {}
    
    for x in np.arange(0,width,1)/0.6: # is this right? should it be width instead of height bc this is x? .6 is a threshold, were scaling up bc this is in cm
        for y in np.arange(0,height,1)/0.6:
            for theta in np.arange(0,120,10)/10:
                
                already_visited[x,y,theta] = 0
                
    return already_visited


def nodes_explored_discrete(): # used to store..?, what's the difference between this and the function above?
    
    exploredcost = {}
    
    for x in np.arange(0,width,1)/0.6: # is this right? should it be width instead of height bc this is x?
        for y in np.arange(0,height,1)/0.6:
            for theta in np.arange(0,120,10)/10:
                
                exploredcost[x,y,theta] = 0

    return exploredcost
                
                
def duplicate_checker(current_node, visited_nodes): # use this function everytime we loop through robot Astar to see if there is a duplicate node
    
    duplicate_nodes = False
    
    if current_node in visited_nodes:
        
        if visited_nodes[current_node] == 1:
            
            duplicate_nodes = True # if the current_node is in visited_nodes, return True for duplicate_nodes
            
        else:
            
            visited_nodes[current_node] = 1 # if the current_node is not in visited_nodes, assign the key current_node in the dictionary visited_nodes
                                            # so next time we call this fn, it will return True for duplicate_nodes
            
    return duplicate_nodes

def roundUp(node):
    
    if node[2] < 0:
        theta_ = 360+round(node[2])
        
    else:
        theta_ = node[2]
   
    round_n = (round(node[0],1), round(node[1],1), theta_)
    
    return round_n

# End of New Code Implementation ***********************

def Heuristic_Distance(current_node, proximity_node):
    
    euclidean_dist = math.sqrt((proximity_node[0] - current_node[0])**2 + (proximity_node[1] - current_node[1])**2)
    
    return euclidean_dist

def obstacle_checker(node, clearance, radius):
    
    IsPresent = False
    
    x = node[0]
    y = node[1]
    
    c = 10
    
    # offset_x= 0
    # offset_y = 0
    
    # offset = clearance + radius
    

    # if ( x - 400)**2 + ( y - 90 )**2 <= 50**2 <= 0: # circle
    #     IsPresent = True

    if (x - 400)**2 + (y - 110)**2 - 50**2 <= 0: # circle
        IsPresent = True
            
    if (x > 150 - c) and (x < 165 + c) and (y > 75 - c) and (y < 200): # left rectangle
        IsPresent = True

    if (x > 250 - c) and (x < 265 + c) and (y < 125 + c) and (y > 0): # right rectangle
        IsPresent = True
        
    if (x <= 5 + c) or (x >= 595 - c) or (y <= 5 + c) or (y >= 195 - c): # boundaries
        IsPresent = True
        
    return IsPresent
            
def newValidNodeChecker(current_node, clearance, radius):
    
    valid = False
    
    if current_node[0] >= 0 or current_node[1] >= 0:
        
        valid = True
        
    if current_node[0] <= width or current_node[1] <= height:
        
        valid = True
        
    if obstacle_checker(current_node, clearance, radius) == True:
        
        valid = True
        
    return valid

def Nodes_per_Action(node, lwheel,rwheel): # used in conjunction with robot_traj_coord to generate children of give node
    
    posX = float(node[0])
    posY = float(node[1])
    theta = float(node[2])
    
    nearest_nodes = []
    
    RPM1 = lwheel
    RPM2 = rwheel
    
    actions = [[0,RPM1],
                [RPM1,0],
                [RPM1,RPM1],
                [0,RPM2],
                [RPM2,0],
                [RPM2,RPM2],
                [RPM1,RPM2],
                [RPM2,RPM1]]
    
    for move in actions:
        
        posXn, posYn, thetan, curvature, xList, yList = robot_traj_coor(posX, posY, theta, move[0], move[1])
        
        nearest_nodes.append((round(posXn,3), round(posYn,3), round(thetan), move[0], move[1], round(curvature,3)))
        
    return nearest_nodes # return list of tuples

def Robot_ASTAR(start_pos, goal_pos, goal_radius, duplicates, clearance, radius, RPM1, RPM2):
    # start_pos = [x, y, theta]
    
    node_goal_bnds = (0,0,0) # the bounds of the node to goal node
    
    Childs_n_Parent = []
 
    duplicates[start_pos] = 1 # assign the starting position a value of 1 to show it has been checked
    
    explored_cost = nodes_explored_discrete() # create a dictionary for every node in the workspace, they're all assigned a cost of zero initially
    explored_cost[start_pos] = 0 # assign the starting node a c2c of 0
    # 
    
    cost2come = {start_pos:0} # G
    cost2go = {start_pos: Heuristic_Distance(start_pos, goal_pos)} # H
    cost = {start_pos: Heuristic_Distance(start_pos, goal_pos)} # F
    
    
    open_path = (cost[start_pos], start_pos) # (distance2goal, [x, y, theta])
    
    open_list = PriorityQueue()
    closed_list = set()
    
    store_OL = set([start_pos]) # start_pos = [x, y, theta]
    exploredPath = {}


    previous_visited = []
    open_list.put(open_path) # (cost, (x, y, theta))
    Parent = {} # contains an adjacent map of all nodes
    current_child_nodes = {start_pos: None}
    
    # Open list created just for Storing the Queue Values
    
    iterations = 0
        
    while len(store_OL) > 0:
        
        current_node = None # node with lowest f() is found
        neighbor_nodes = []
        cost_current = None
        
        for node in store_OL:
            
            H = Heuristic_Distance(node, goal_pos) # C2G
            pos_index = (node[0], node[1], node[2]) # (x, y, theta)
            
            if current_node is None or cost[pos_index] + H*1.35 < cost_current: # weighted a star search
                
                cost_current = cost[pos_index] # get the current cost using the pos_index key, we updat the cost dictionary in the 2nd to last line of this func
                current_node = node
            
        rnd_curNode = roundUp(current_node)	  
        rnd_curCost =  float(explored_cost[rnd_curNode])
        
        if iterations > 0:
            
            current_child_nodes[current_node] = exploredPath[current_node]
            Parent_node = [item for item in Childs_n_Parent if item[1] == current_node]
            Parent = Parent_node[0][0]
            previous_visited.append((Parent, current_node))
            
        
        if Heuristic_Distance(current_node, goal_pos) <= goal_radius:
            
            node_goal_bnds = current_node
            # #print(" explored_cost: ", explored_cost)
            #print(" Goal is Reached", iterations)
            # #print(" type(current_child_nodes): ", type(current_child_nodes), "\n current_child_nodes: ", current_child_nodes)
            # #print(" type(explored_cost): ", type(explored_cost), "len(explored_cost)", len(explored_cost)) # \n explored_cost: ", explored_cost)
            # #print("closed_list: ", closed_list)
            # #print("\nstore_OL: ", store_OL)
            # #print("\nopen_list: ", open_list)
            # #print("\nclosed_list: ", closed_list)
            # #print("\nexplored_cost: ", explored_cost)

            return current_child_nodes, node_goal_bnds, previous_visited

        store_OL.remove(current_node)
        closed_list.add(current_node)
        
        # find_closest_neighbors(explored_cost, (20, 20, 30), 5) # probalistic method for generating search graph
        neighbor_nodes = Nodes_per_Action(current_node, RPM1, RPM2) # Robot Moves, create children, action based method for generating the search graph
        
        #             (x, y, theta)
        cur_index = (current_node[0], current_node[1], current_node[2])

        iterations = iterations + 1
        
        for new_pos_node in neighbor_nodes:
            
            if newValidNodeChecker(new_pos_node, clearance, radius) == False:
                continue # if the new child is in the obstacle space, exit the for loop

            if new_pos_node in closed_list:
                continue # if the new childs location has been explored before, exit the for loop

            new_node_created = roundUp(new_pos_node)
            cost2come_updated = cost2come[cur_index] + new_pos_node[5]
            rnd_newCost = rnd_curCost + new_pos_node[5]
            
            if duplicate_checker(new_node_created, duplicates) == False:
    

                # open_list.put(new_pos_node)
                store_OL.add(new_pos_node)
                
                Childs_n_Parent.append((current_node, new_pos_node))
               
                
            elif cost2come_updated >= explored_cost[new_node_created]:
                
                continue # if the new C2C > OG C2C, exit the for loop
            
            explored_cost[new_node_created] = rnd_newCost
            exploredPath[new_pos_node] = current_node
            
            c_indx = (new_pos_node[0], new_pos_node[1], new_pos_node[2])
            cost2go_updated = Heuristic_Distance(new_pos_node, goal_pos) #H
            cost2come[c_indx] = cost2come_updated #G
            cost[c_indx] = cost2come[c_indx] + cost2go_updated # F = G + H = total cost
            Childs_n_Parent.append((current_node, new_pos_node))

# this function creates a random sample within the defined map and checks if its in the obstacle space
def random_sample(clearance, radius):

    width = 600
    height = 200

    state = False
    while state == False:
        randomNode = (randint(0, width), randint(0, height), randint(0, 360), 0, 0, 0)
        # if newValidNodeChecker(randomNode, clearance, radius) == True and radius >= euclidean_dist(current_node[0], current_node[1], randomNode[0], randomNode[1]):
        if newValidNodeChecker(randomNode, clearance, radius) == True:
            state = True
    #print("\nrandomNode: ", randomNode)
    
    return randomNode

# returns the 1 closest node currently on the tree to the sample
def find_closest_neighbors(tree, sample, metric_function=euclidean_dist):
    #print("\n*******find_closest_neighbors to sample*******")
    #print("using tree: ", tree)
    #print("with sample: ", sample)

    nodesWithinDistance = []
    x1 = sample[0]
    y1 = sample[1]

    min_distance = float('inf')
    closest_value = None

    for key, value in tree.items():
        if None in key:
            continue
        if isinstance(value, list): # check if there are multiple nodes in the key
            for node in value:
                x2, y2, th2, rpm1, rpm2, c2c = node
                if metric_function == euclidean_dist:
                    distance = euclidean_dist(x1, y1, x2, y2)
                elif metric_function == manhattan_dist:
                    distance = manhattan_dist(x1, y1, x2, y2)
                elif metric_function == diagonal_dist:
                    distance = diagonal_dist(x1, y1, x2, y2)
                #print("node: ", node, "is at a distance: ", round(distance, 2), "of the sample: ", sample)

                if distance < min_distance:
                    closest_value = node
                    min_distance = distance
        else:
            x2, y2, th2, rpm1, rpm2, c2c = value
            if metric_function == euclidean_dist:
                distance = euclidean_dist(x1, y1, x2, y2)
            elif metric_function == manhattan_dist:
                distance = manhattan_dist(x1, y1, x2, y2)
            elif metric_function == diagonal_dist:
                distance = diagonal_dist(x1, y1, x2, y2)

            if distance < min_distance:
                closest_value = value
                min_distance = distance

    if closest_value is not None:
        nodesWithinDistance.append(closest_value)

    #print("closest node to sample: ", nodesWithinDistance)
    #print("*******find_closest_neighbors*******\n")
    return nodesWithinDistance

# from the 1 neighbor found in function above, 8 child nodes are generated, the child closest to the sample is selected
# and returned as x_new
def steer(sample, neighbors, ul, ur, metric_function=euclidean_dist):
    # neighbors should be from find_closest_neighbors
    #print("\n*******steer*******")

    new_node_dict = {}
    # neighbors_and_children = {}

    for neighbor in neighbors:
        neighborChildren = Nodes_per_Action(neighbor, ul, ur)
        #print("neighborChildren: ", neighborChildren)
        neighbor = [neighbor[0], neighbor[1], neighbor[2], neighbor[3], neighbor[4], round(neighbor[5])]
        #print("neighbor: ", neighbor)
        for child in neighborChildren:
            # neighbors_and_children[neighbor] = child
            # #print("child: ", child)
            x_n, y_n, theta_n, rpmLeft, rpmRight, curvature = child
            if metric_function == euclidean_dist:
                distance = euclidean_dist(sample[0], sample[1], x_n, y_n)
            elif metric_function == manhattan_dist:
                distance = manhattan_dist(sample[0], sample[1], x_n, y_n)
            elif metric_function == diagonal_dist:
                distance = diagonal_dist(sample[0], sample[1], x_n, y_n)
            new_node_dict[(x_n, y_n, theta_n, rpmLeft, rpmRight)] = (round(distance, 2), neighbor)
    # the keys in this dictionary are the children created from all the neighbor nodes, their value is the euclidean distance from the sample
    # #print("neighbors_and_children: ", neighbors_and_children)
    #print("nodes generated from neighbor: ", new_node_dict)

    # x_new = min(new_node_dict.items(), key = lambda x: x[1])
    # #print("x_new: ", x_new)
    x_new = min(new_node_dict, key = new_node_dict.get) # this is the node we are trying to get to bc it's the closest to the tree
    #print("x_new: ", x_new)
    x_new_value = new_node_dict[x_new]
    # #print("x_new_value: ", x_new_value)
    tree_node = x_new_value[1]
    x_new = ((x_new[0]), x_new[1], x_new[2], x_new[3], x_new[4])
    #print("x_new after int(): ", x_new)
    #print("tree_node: ", tree_node)

    # find action set taken from tree_node to sample
    RPM1 = ul
    RPM2 = ur

    return x_new    

# this function looks through the tree for any nodes within a defined threshold of x_new
def near(tree, x_new, metric_function=euclidean_dist):
    
    #print("\n***********near**************")  
    #print("sample: ", x_new)
    #print("tree: ", tree)
    threshold = 20
    near_nodes = []
    
    for key, value in tree.items():
        if None in key:
            continue
        if isinstance(value, tuple):
            nodes = [value]
        else:
            nodes = value
        
        for node in nodes:
            x2, y2, th2, rpm1, rpm2, c2c = node
            if metric_function == euclidean_dist:
                distance = euclidean_dist(x_new[0], x_new[1], x2, y2)
            elif metric_function == manhattan_dist:
                distance = manhattan_dist(x_new[0], x_new[1], x2, y2)
            elif metric_function == diagonal_dist:
                distance = diagonal_dist(x_new[0], x_new[1], x2, y2)
            
            if distance < threshold:
                near_nodes.append(node)

    #print("nodes within threshold of: ", threshold, "to sample: ", x_new, "are: \n", near_nodes)
    #print("***********near**************\n")
    
    return near_nodes

# this function looks through near_nodes list and chooses the node that can access x_new via one of its 8 action sets
def choose_parent(current_node, near_nodes, goal_node, ul, ur, metric_function=euclidean_dist):
    
    #print("\n*******choose_parent*******")
    best_cost = float("inf")
    prev_dist = float("inf")
    best_c2g = float("inf")
    best_parent = None

    for node in near_nodes:

        children = Nodes_per_Action(node, ul, ur)
        for child in children:
            if child[0] == current_node[0] and child[1] == current_node[1]:
                best_parent = node
        if metric_function == euclidean_dist:
            dist = euclidean_dist(current_node[0], current_node[1], node[0], node[1]) # distance between node in near_nodes and sample
            cost2goal = euclidean_dist(node[0], node[1], goal_node[0], goal_node[1])
        elif metric_function == manhattan_dist:
            dist = manhattan_dist(current_node[0], current_node[1], node[0], node[1]) # distance between node in near_nodes and sample
            cost2goal = manhattan_dist(node[0], node[1], goal_node[0], goal_node[1])
        elif metric_function == diagonal_dist:
            dist = diagonal_dist(current_node[0], current_node[1], node[0], node[1]) # distance between node in near_nodes and sample
            cost2goal = diagonal_dist(node[0], node[1], goal_node[0], goal_node[1])
        cost2come = dist + node[5]
        total_cost = cost2come + cost2goal
        # #print("node in near_nodes: ", node, "has a c2c: ", round(cost2come, 3), "and a c2g: ", round(cost2goal, 3), "which gives a total cost of: ", round(total_cost, 3))
        # #print("node: ", node, "is at a distance: ", round(dist, 3), "from the sample: ", current_node)

        # if best_parent is None or dist < best_cost:
        #     best_cost = dist
        #     best_parent = node
        #     #print("updating best_parent to: ", best_parent)


        # if dist <= prev_dist:
        #     best_cost = total_cost
        #     best_parent = node
        #     #print("updating best_parent to: ", best_parent)

        # prev_dist = dist
    # #print("\nbest parent/z_min for x_new/sample: ", current_node, " is: ", best_parent, "picking this parent gives x_new a total cost of: ", round(best_cost,2))

        # if total_cost <= best_cost:
        #     best_parent = node
        #     best_cost = total_cost
    # #print("\nbest parent/z_min for x_new/sample: ", current_node, " is: ", best_parent, "picking this parent gives x_new a total cost of: ", round(best_cost,2))


        # if cost2goal <= best_c2g:
        #     best_parent = node
        #     best_c2g = cost2goal
    # #print("\nbest parent/z_min for x_new/sample: ", current_node, " is: ", best_parent, "picking this parent gives x_new a c2g of: ", round(best_c2g, 2))
    #print("*******choose_parent*******\n")
    
    return best_parent

# this function inserts the child into the tree, also calculates the C2C
def insert_node(tree, parent, child, metric_function=euclidean_dist):

    #print("\n******insert_node********")
    parent = (parent[0], parent[1], parent[2], parent[3], parent[4], parent[5])

    if parent in tree:
        if metric_function == euclidean_dist:                                                                                                # add the cost from the parent to x_new to the parent cost
            tree[parent].append( [ child[0], child[1], child[2], child[3], child[4], parent[5] + round(euclidean_dist(parent[0], parent[1], child[0], child[1]), 3) ] )
        elif metric_function == manhattan_dist:
            tree[parent].append( [ child[0], child[1], child[2], child[3], child[4], parent[5] + round(manhattan_dist(parent[0], parent[1], child[0], child[1]), 3) ] )
        elif metric_function == diagonal_dist:
            tree[parent].append( [ child[0], child[1], child[2], child[3], child[4], parent[5] + round(diagonal_dist(parent[0], parent[1], child[0], child[1]), 3) ] )
    else:
        #print("parent is not in tree, creating new parent key and adding sample as child")
        parent = (parent[0], parent[1], parent[2], parent[3], parent[4], parent[5])
        tree[parent] = []
        if metric_function == euclidean_dist:
            tree[parent].append( [ child[0], child[1], child[2], child[3], child[4], round(parent[5] + euclidean_dist(parent[0], parent[1], child[0], child[1]), 3) ] )
        elif metric_function == manhattan_dist:
            tree[parent].append( [ child[0], child[1], child[2], child[3], child[4], round(parent[5] + manhattan_dist(parent[0], parent[1], child[0], child[1]), 3) ] )
        elif metric_function == diagonal_dist:
            tree[parent].append( [ child[0], child[1], child[2], child[3], child[4], round(parent[5] + diagonal_dist(parent[0], parent[1], child[0], child[1]), 3) ] )
    #print("tree: ", tree)
    #print("******insert_node********\n")
    
    return tree

def rrt(start_node, clearance, N, ul, ur, goal_node, metric_function=euclidean_dist):
    # N is number of samples we want to try
    # clearance is the same clearance from user_goals

    time_start = time.time()
    tree = {(float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf')): start_node}
    iteration = 0
    #print("using ", N, "samples")
    plt.ion() # so we can visualize the tree being created

    if metric_function == euclidean_dist:
        print(" using euclidean distance as metric function")
        for i in range(N):
            iteration += 1
            #print("***************************************iteration: ", iteration, "**************************************************************")
            random_node = random_sample(clearance=.6, radius=10) # create random sample
            #print("tree: ", tree)
            neighbors = find_closest_neighbors(tree, random_node) # find closest neighbor of that sample
            x_new = steer(random_node, neighbors, ul, ur) # generate child nodes from closest neighbor, x_new is the child node closest to the sample

            if obstacle_checker(x_new, clearance=5, radius = 5) == False:
                # if x_new is not in the obstacle space then...
                closer_neighbors = near(tree, x_new) # find all neighbors of x_new within a defined threshold
                z_min = choose_parent(x_new, closer_neighbors, goal_node, ul, ur) # pick the node who can reach x_new using 1 of the 8 action sets
                tree = insert_node(tree, z_min, x_new) # insert z_min as the parent, and x_new as the child, into the tree
                #print("x_new: ", x_new)
                rpms = (0, 0, 0, x_new[3], x_new[4])
                plt.plot([z_min[0]], [z_min[1]], 'x', label = 'child')
                plt.plot([x_new[0]], [x_new[1]], '+', label = 'parent')
                Curved_Line_Robot(z_min, rpms, "blue")
                
                if euclidean_dist(x_new[0], x_new[1], goal_node[0], goal_node[1]) <= 15:
                    # print("tree: ", tree)
                    time_end = time.time()
                    time_taken = time_end - time_start

                    goal = list(tree.values())[-1]
                    print("goal node:", goal, "reached in", iteration, "iterations which took", time_taken, "seconds, has a cost of", goal[0][5])
                    path_list = pathtrace(goal, tree, start_node)
                    path_list[-1] = tuple(path_list[-1][0])

                    # in this loop we shift all the rpms of the current node to the previous node
                    # so path_list can have the correct rpms with Curved_Line_Robot to plot the path from the current to the next node
                    for i in range(1, len(path_list)):
                        path_list[i-1] = path_list[i-1][:3] + path_list[i][3:5] + path_list[i-1][5:]
                    path_list.pop()

                    for node in path_list:
                        Curved_Line_Robot(node, node, 'red', 1.8)
                    while True:
                        if plt.waitforbuttonpress():
                            if plt.get_current_fig_manager().toolbar.mode == 'q':
                                plt.close('all')
                                return tree

            plt.draw() # redraw the plot with the new random node
            plt.pause(.0000000000000001)

    elif metric_function == manhattan_dist:
        print(" using manhattan distance as metric function")
        for i in range(N):
            iteration += 1
            #print("***************************************iteration: ", iteration, "**************************************************************")
            random_node = random_sample(clearance=.6, radius=10) # create random sample
            #print("tree: ", tree)
            neighbors = find_closest_neighbors(tree, random_node, metric_function=manhattan_dist) # find closest neighbor of that sample
            x_new = steer(random_node, neighbors, ul, ur, manhattan_dist) # generate child nodes from closest neighbor, x_new is the child node closest to the sample

            if obstacle_checker(x_new, clearance=5, radius = 5) == False:
                # if x_new is not in the obstacle space then...
                closer_neighbors = near(tree, x_new, manhattan_dist) # find all neighbors of x_new within a defined threshold
                z_min = choose_parent(x_new, closer_neighbors, goal_node, ul, ur, manhattan_dist) # pick the node who can reach x_new using 1 of the 8 action sets
                tree = insert_node(tree, z_min, x_new, manhattan_dist) # insert z_min as the parent, and x_new as the child, into the tree
                #print("x_new: ", x_new)
                rpms = (0, 0, 0, x_new[3], x_new[4])
                plt.plot([z_min[0]], [z_min[1]], 'x', label = 'child')
                plt.plot([x_new[0]], [x_new[1]], '+', label = 'parent')
                Curved_Line_Robot(z_min, rpms, "blue")
                
                if manhattan_dist(x_new[0], x_new[1], goal_node[0], goal_node[1]) <= 15:
                    # print("tree: ", tree)
                    time_end = time.time()
                    time_taken = time_end - time_start

                    goal = list(tree.values())[-1]
                    print("goal node:", goal, "reached in", iteration, "iterations which took", time_taken, "seconds, has a cost of", goal[0][5])
                    path_list = pathtrace(goal, tree, start_node)
                    path_list[-1] = tuple(path_list[-1][0])

                    # in this loop we shift all the rpms of the current node to the previous node
                    # so path_list can have the correct rpms with Curved_Line_Robot to plot the path from the current to the next node
                    for i in range(1, len(path_list)):
                        path_list[i-1] = path_list[i-1][:3] + path_list[i][3:5] + path_list[i-1][5:]
                    path_list.pop()

                    for node in path_list:
                        Curved_Line_Robot(node, node, 'red', 1.8)
                    while True:
                        if plt.waitforbuttonpress():
                            if plt.get_current_fig_manager().toolbar.mode == 'q':
                                plt.close('all')
                                return tree
            plt.draw() # redraw the plot with the new random node
            plt.pause(.0000000000000001)
                            
    elif metric_function == diagonal_dist:
        print("using diagonal distance as metric function")
        for i in range(N):
            iteration += 1
            #print("***************************************iteration: ", iteration, "**************************************************************")
            random_node = random_sample(clearance=.6, radius=10) # create random sample
            #print("tree: ", tree)
            neighbors = find_closest_neighbors(tree, random_node, metric_function=diagonal_dist) # find closest neighbor of that sample
            x_new = steer(random_node, neighbors, ul, ur, diagonal_dist) # generate child nodes from closest neighbor, x_new is the child node closest to the sample

            if obstacle_checker(x_new, clearance=5, radius = 5) == False:
                # if x_new is not in the obstacle space then...
                closer_neighbors = near(tree, x_new, diagonal_dist) # find all neighbors of x_new within a defined threshold
                z_min = choose_parent(x_new, closer_neighbors, goal_node, ul, ur, diagonal_dist) # pick the node who can reach x_new using 1 of the 8 action sets
                tree = insert_node(tree, z_min, x_new, diagonal_dist) # insert z_min as the parent, and x_new as the child, into the tree
                #print("x_new: ", x_new)
                rpms = (0, 0, 0, x_new[3], x_new[4])
                plt.plot([z_min[0]], [z_min[1]], 'x', label = 'child')
                plt.plot([x_new[0]], [x_new[1]], '+', label = 'parent')
                Curved_Line_Robot(z_min, rpms, "blue")
                
                if manhattan_dist(x_new[0], x_new[1], goal_node[0], goal_node[1]) <= 15:
                    # print("tree: ", tree)
                    time_end = time.time()
                    time_taken = time_end - time_start

                    goal = list(tree.values())[-1]
                    print("goal node:", goal, "reached in", iteration, "iterations which took", time_taken, "seconds, has a cost of", goal[0][5])
                    path_list = pathtrace(goal, tree, start_node)
                    path_list[-1] = tuple(path_list[-1][0])

                    # in this loop we shift all the rpms of the current node to the previous node
                    # so path_list can have the correct rpms with Curved_Line_Robot to plot the path from the current to the next node
                    for i in range(1, len(path_list)):
                        path_list[i-1] = path_list[i-1][:3] + path_list[i][3:5] + path_list[i-1][5:]
                    path_list.pop()

                    for node in path_list:
                        Curved_Line_Robot(node, node, 'red', 1.8)
                    while True:
                        if plt.waitforbuttonpress():
                            if plt.get_current_fig_manager().toolbar.mode == 'q':
                                plt.close('all')
                                return tree
                
            # else: 
            #     print("x_new: ", x_new, "is in obstacle space, not inserting!")

            plt.draw() # redraw the plot with the new random node
            plt.pause(.0000000000000001)

    plt.ioff()
    plt.show()

    return tree


def pathtrace(goal, tree, start_node):
    #print("\n***********************pathtrace********************************")
    #print("tree: ", tree)
    path = [goal] # add the goal node to the path
    current = goal # going to use current to find its key
    visited = set() # add keys explored
    # #print("path: ", path, "is of type: ", type(path))
    #print("current: ", current, "is of type: ", type(current))
    # #print("visited: ", visited, "is of type: ", type(visited))
    i = 0
    while current != start_node:
        if i == 0:
            current = current[0] # needed bc the first current (goal) is a nested list
        if i >= 1:
            current = list(current)
            #print("current after list: ", current, type(current))
        #print("current inside while loop: ", current, "is of type: ", type(current))
        # visited.add(tuple(current))  # add the current node to the visited list
        # #print("visited: ", visited)
        i += 1
        for parent, children in tree.items(): 
            # time.sleep(.05)
            # #print("\n")
            if current in children:
                # if current is a value in the tree dictionary, do the following:
                #print("\n", current, "is in", parent, "key")
                # if tuple(parent) in visited:  # check if the parent has already been visited
                #     return None  # cycle detected, return None
                path.append(parent)
                #print("parent: ", parent, "is of type: ", type(parent))
                current = parent
                break
    #print("***********************pathtrace********************************\n")
    return path[::-1]


def backtrack(visited_nodes):

    goal_node = list(visited_nodes.keys())[-1] # get the last key in the dictionary bc its the goal node
    parent_node = visited_nodes[goal_node] # get the value of the last key, this is the last keys parent node
    path = [goal_node]

    while parent_node is not None: # backtrack using the parent nodes until value = none which means we're at the starting node
        path.append(parent_node) # add the node to the path each iteration
        parent_node = visited_nodes[parent_node] # update the parent node

    path.reverse()

    x_coords = [node[0] for node in path]
    y_coords = [node[1] for node in path]
    # #print("x_coords: ", x_coords)
    # #print("y_coords: ", y_coords)

    return x_coords, y_coords

def Curved_Line_Robot(nodePar, nodeCh, linecolor, linewidth=0.6):
    
    t = 0 
    R = 0.38
    L = 3.540
    dt = 0.1
    
    xn = nodePar[0]
    yn = nodePar[1]
    
    theta_rad = 3.14 * nodePar[2] / 180
    
    UL = nodeCh[3]
    UR = nodeCh[4]
    
    while t < 1:
        
        t = t + dt
        xs = xn
        ys = yn
        
        xn += (R / 2) * (UL + UR) * m.cos(theta_rad) * dt
        yn += (R / 2) * (UL + UR) * m.sin(theta_rad) * dt
        theta_rad += (R / L) * (UR - UL) * dt
        
        plt.plot([xs, xn],[ys, yn], color=linecolor, linewidth=linewidth)



    theta_deg = 180 * (theta_rad) / 3.14
    
    if theta_deg >= 360:
        
        theta_deg = (theta_deg - 360)
        
    if theta_deg <= -360:
        theta_deg = (theta_deg + 360)
        
    return xn, yn, theta_deg

def bufImage(): # convert a matplotlib figure to an opencv object
    Obs_space.fig.canvas.draw() # 
    mapImg = np.frombuffer(Obs_space.fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(Obs_space.fig.canvas.get_width_height()[::-1] + (3,))
    mapImg = cv2.cvtColor(mapImg,cv2.COLOR_RGB2BGR)
    return mapImg

def Simulated_BotShow(nodes, xCoords, yCoords):
    
    #print(" The Simulation has Started")
    # #print(" nodes: ", nodes)
    
    for node in nodes:
        
        Curved_Line_Robot(node[0], node[1], "orange")
        mapImg = bufImage()


        if cv2.waitKey(1) == ord('q'):
            
            exit()
        cv2.imshow('Robot - A Star', mapImg)

    plt.plot(xCoords, yCoords, c='blue', marker = '.')
    mapImg = bufImage()
    cv2.imshow('Robot - A Star', mapImg)

def polynomial(x, *coeffs):
    y = []
    for xx in x:
        y.append(sum([coeffs[i] * xx**i for i in range(len(coeffs))]))
    return y

def steering(x_data, y_data, current_node, near_node):
    
    angle_radians = np.radians(current_node[1])
    # create a horizontal line at the given angle to be able to flip x_data and y_data over this line
    # this flipped data will be used to create a lower bound to check if the current_node is above it
    slope = np.tan(angle_radians)
    y_int = y_data[0] - slope * x_data[0]
    x_line = np.linspace(np.min(x_data), np.max(x_data), len(x_data))
    y_line = slope * x_line + y_int

    x_data_flipped = (x_data + (y_data - 2*y_line) / (slope**2 + 1))
    # y_data_flipped = (y_line + slope*(x_data + (y_data - 2*y_line) / (slope**2 + 1)))
    y_data_flipped = (2*y_line - y_data)
    plt.plot(x_data, y_data, 'o', label = 'Original Data')
    plt.plot(x_data, y_data_flipped, 'x', label = "flipped data")
    plt.plot(x_line, y_line, label = 'line at the angle')
    plt.legend()
    plt.show()
    
    p0 = [1, 1, 1] # used provide initial guess for parameters
    coeffs, _ = curve_fit(polynomial, x_data, y_data, p0)
    flipped_coeffs, _ = curve_fit(polynomial, x_data, y_data_flipped, p0)
    #print("coeffs: ", coeffs, "\nequation is: ", coeffs[0], coeffs[1], "x +", coeffs[2], "x**2" )
    #print("flipped_coeffs: ", flipped_coeffs, "\nequation is: ", flipped_coeffs[0], flipped_coeffs[1], "x +", flipped_coeffs[2], "x**2" )

    # the following checks if near_node is within the workspace of the mobile robot, we have to check this bc its a differential drive
    # if y <= c1 + c2x + c3x^2 and x_data[0] <= x <= x_data[-1]
    if current_node[1] <= coeffs[0] + coeffs[1] * current_node[0] + coeffs[2] * current_node[0] ** 2 \
        and current_node [1] >= flipped_coeffs[0] + flipped_coeffs[1] * current_node[0] + flipped_coeffs[2] * current_node[0]**2 \
        and current_node[0] >= x_data[0] and current_node[0] <= x_data[-1]:
        #print("\ncurrentNode", current_node, "is in bounds")
        return False
    else:
        #print("currentNode", current_node, "is out of bounds")
        return True

def user_goals():
    
    start_positions = []
    goal_positions = []

    if int(input(" enter 1 for predefined, 2 to define yourself : ")) == 1:
        start_positions = [5.0, 5.0, 45.0]
        goal_positions = [20.0, 20.0, 45.0]
        speed1 = 35
        speed2 = 30
        Robot_Radius = 8.0
        Map_C = 0.6
        #print(" Start Position : ", start_positions, "\n Goal Position : ", goal_positions)
        #print(" RPM1 : ", speed1, "RPM2 : ", speed2)
        #print(" Robot Radius : ", Robot_Radius, "Obstacle Clearance : ", Map_C)

    
    else: 
        print(" Please enter values for Start Position [x y theta], e.g [5 5 30]")
        
        for i in range(3):
            
            start_positions.append(float(input(" ")))
        print("start_positions: ", start_positions, "type(start_positions): ", type(start_positions))
        print(" Initial Start is: ", start_positions)
        print("\n Please enter values for Goal Position [x y theta, e.g [7 8 60]")
        
        for i in range(3):
            goal_positions.append(float(input(" ")))
        speed1 = float(input(" Please enter values for RPM1 (value: 5 to 15) : "))
        speed2 = float(input(" Please enter values for RPM2 (value: 15 to 30) : "))
        Robot_Radius = float(input(" Please input the Robot radius (e.g. 10): "))
        Map_C = float(input(" Please input the Obstacle Clearance (e.g. 0.1 - 0.8): "))
    
    return start_positions, goal_positions, speed1, speed2, Robot_Radius, Map_C


def prompt():

    print("\n start node = ? \n goal node = ? \n clearance = ? \n samples = ? \n rpm1, rpm2 = ? ")
    option = int(input(" Enter 1 for predefined, 2 for user defined: "))
    while option != 1 and option != 2:
        option = int(input(" \nEnter 1 for predefined, 2 for user defined: "))

    if option == 1:
        startNode = (30, 30, 45, 0, 0, 0)
        goalNode = (530, 100, 45, 0, 0, 0)
        print(" Using start node:", startNode, "and goal node:", goalNode, "with 2000 samples, RPM1 = 25, RPM2 = 30 and a clearance of 0")
        
        metric_function = -1
        while not (1 <= metric_function <= 3):       
            metric_function = int(input("\n This program has 3 available metric functions\n Enter 1 for euclidean, 2 for manhattan, 3 for diagonal: "))
            if metric_function == 1:
                tree = rrt(startNode, 0, 2000, 25, 30, goalNode, euclidean_dist) # using manhattan distance
            if metric_function == 2:
                tree = rrt(startNode, 0, 2000, 25, 30, goalNode, manhattan_dist) # using euclidean distance
            if metric_function == 3:
                tree = rrt(startNode, 0, 2000, 25, 30, goalNode, diagonal_dist) # using euclidean distance

    if option == 2:
        start_positions = []
        goal_positions = []

        # gather user input
        print("\n Please enter values for Start Position [x y theta], e.g [15 15 30]:")
        for i in range(3):
            start_positions.append(int(input(" ")))
        start_positions = tuple(start_positions) + (0, 0, 0)

        print(" Please enter values for the Goal Position [x y theta], e.g [350, 100, 0]:")
        for i in range(3):
            goal_positions.append(int(input(" ")))
        goal_positions = tuple(goal_positions) + (0, 0, 0)

        samples = int(input(" Enter the max amount of samples you want to use (at least 1000 is recommended): "))
        clearance = int(input(" Enter your desired obstacle clearace: "))
        
        rpmLeft = -1
        while not (0 < rpmLeft <= 45):
            rpmLeft = int(input(" Enter your desired RPM1 (between 1 and 45): "))
        rpmRight = -1
        while not (0 < rpmRight <= 45):
            rpmRight = int(input(" Enter your desired RPM2 (between 1 and 45): "))
        print("startNode:", start_positions)
        print("endNode:", goal_positions)

        metric_function = -1
        while not (1 <= metric_function <= 3):
            metric_function = int(input("\n This program has 3 available metric functions\n Enter 1 for euclidean, 2 for manhattan, 3 for diagonal: "))
            if metric_function == 1:
                tree = rrt(start_positions, clearance, samples, rpmLeft, rpmRight, goal_positions, euclidean_dist) # using manhattan distance
            if metric_function == 2:
                tree = rrt(start_positions, clearance, samples, rpmLeft, rpmRight, goal_positions, manhattan_dist) # using manhattan distance
            if metric_function == 3:
                tree = rrt(start_positions, clearance, samples, rpmLeft, rpmRight, goal_positions, diagonal_dist) # using manhattan distance
            
###################### Testing ######################
plt.close('all')

prompt()

startNode = (30, 30, 45, 0, 0, 0)
goalNode = (350, 100, 45, 0, 0, 0)
# goalNode = (60, 60, 45, 0, 0, 0)

# print("euclidean dist: ", euclidean_dist(startNode[0], startNode[1], goalNode[0], goalNode[1]))
# print("manhattan dist: ", manhattan_dist(startNode[0], startNode[1], goalNode[0], goalNode[1]))
# print("diagonal dist: ", diagonal_dist(startNode[0], startNode[1], goalNode[0], goalNode[1]))

# # rrt ( start_node, clearance, N, ul, ur, goal_node)
# # tree = rrt(startNode, 0, 2000, 20, 30, goalNode) # using default euclidean distance
# tree = rrt(startNode, 0, 2000, 25, 30, goalNode, diagonal_dist) # using manhattan distance

# #print("tree: ", tree)

# create_an_astar(tree, startNode, goalNode, 5, 10, .5, 8)

# xn, yn, theta_list = Curved_Line_Robot((1, 1, 15, 30), (80, 80, 15, 15, 15), "blue")
# #print("x_list: ", xn)
# #print("y_list", yn)
# # plt.plot(x_list, y_list, 'x')
# plt.show()

###################### Testing ######################

# goal_radius = 0.5

# start, goal, RPM1, RPM2, R, map_c = user_goals()

# start = tuple(start)
# goal = tuple(goal)



# duplicate_nodes = nodes_visited() # initialize multidimensional array with a value of zero, this will be used to check for duplicate nodes
        
# visited_nodes, node_goal_bnds, nodes_path = Robot_ASTAR(start, goal, goal_radius, duplicate_nodes, R, map_c, RPM1, RPM2)
# # i think we can use visited_nodes / current_child_nodes to backtrack
# xCoords, yCoords = backtrack(visited_nodes)


# # #print("nodes_path: ", nodes_path)
# Obs_space = ObsMap(map_c, R)                                        # Creating an instance of the Obstacle Space Object 
# goal_circ = plt.Circle((goal[0],goal[1]), radius=goal_radius, color='#F0DB4F')    # Drawing a goal threshold area in the map
# Obs_space.ax.add_patch(goal_circ)                            # Adding goal circle drawing to map
# goal_circ = plt.Circle((start[0],start[1]), radius=0.1, color='#333399')    # Drawing a goal threshold area in the map
# Obs_space.ax.add_patch(goal_circ)   


# Simulated_BotShow(nodes_path, xCoords, yCoords)


# if cv2.waitKey(0):
    
#     exit()
    
# cv2.destroyAllWindows()
                            
               
