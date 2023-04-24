import cv2
import numpy as np
from queue import PriorityQueue
import time
import matplotlib.pyplot as plt
import math
from copy import deepcopy
import math as m
from random import randint

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






def user_goals():
    
    start_positions = []
    goal_positions = []

    if int(input(" enter 1 for predefined, 2 to define yourself : ")) == 1:
        start_positions = [5.0, 5.0, 30.0]
        goal_positions = [80.0, 80.0, 30.0]
        speed1 = 5.0
        speed2 = 15.0
        Robot_Radius = 8.0
        Map_C = 0.6
        print(" Start Position : ", start_positions, "\n Goal Position : ", goal_positions)
        print(" RPM1 : ", speed1, "RPM2 : ", speed2)
        print(" Robot Radius : ", Robot_Radius, "Obstacle Clearance : ", Map_C)

    
    else: 
        print(" Please enter values for Start Position [x y theta], e.g [5 5 30]")
        
        for i in range(3):
            
            start_positions.append(float(input(" ")))
        # print("start_positions: ", start_positions, "type(start_positions): ", type(start_positions))
            
        print(" Initial Start is: ", start_positions)
        
        
        print("\n Please enter values for Goal Position [x y theta, e.g [7 8 60]")
        
        
        for i in range(3):
            
            goal_positions.append(float(input(" ")))
            
        print(" Final Goal is: ", goal_positions)
        
        speed1 = float(input(" Please enter values for RPM1 (value: 5 to 15) : "))
        
            
        speed2 = float(input(" Please enter values for RPM2 (value: 15 to 30) : "))
        
            
        Robot_Radius = float(input(" Please input the Robot radius (e.g. 10): "))
        
        Map_C = float(input(" Please input the Obstacle Clearance (e.g. 0.1 - 0.8): "))
    
        
    
    return start_positions, goal_positions, speed1, speed2, Robot_Radius, Map_C


    



# # Robot_Radius = 5

threshold = 0.5

# # Test Inputs Starts/Goal


# New Code Implementation ***********************



def euclidean_dist(xi, yi, xf, yf):
    
    e_dist = m.sqrt((xf - xi)**2 + (yf - yi)**2)
    
    return e_dist


def robot_traj_coor(x_init,y_init, theta, vL, vR):
    
    t_init = 0
    R_tire = 0.38
    length = 3.54
    t_delta = 0.1
    
    xn = x_init
    yn = y_init
    
    theta_rad = 3.14 * theta / 180
    
    curvature = 0
    
    while t_init < 1:
        
        t_init = t_init + t_delta
        
        x_s = xn
        y_s = yn
        
        
        # Three Equations UL, UR, Theta
        xn += (R_tire / 2) * (vL + vR) * m.cos(theta_rad)* t_delta
        yn += (R_tire / 2) * (vL + vR) * m.sin(theta_rad)* t_delta
        theta_rad += (R_tire / length) * (vR - vL) * t_delta
        

        curvature += euclidean_dist(x_s, y_s, xn, yn)
        
        
        
    theta_deg = 180*(theta_rad) / 3.14
    
    if theta_deg >= 360:
        
        theta_deg = (theta_deg - 360)
        
    if theta_deg <= -360:
        
        theta_deg = (theta_deg + 360)
        
    return xn, yn, theta_deg, curvature


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
    

    if ( x - 400)**2 + ( y - 90 )**2 <= 50**2 <= 0: # circle
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
        
        posXn, posYn, thetan, curvature = robot_traj_coor(posX, posY, theta, move[0], move[1])
        
        nearest_nodes.append((round(posXn,3), round(posYn,3), round(thetan), move[0], move[1], round(curvature,3)))
        
    return nearest_nodes # return list of tuples

def Robot_ASTAR(start_pos, goal_pos, goal_radius, duplicates, clearance, radius, RPM1, RPM2):
    # start_pos = [x, y, theta]
    
    node_goal_bnds = (0,0,0) # the bounds of the node to goal node
    
    Childs_n_Parent = []
 
    duplicates[start_pos] = 1 # assign the starting position a value of 1 to show it has been checked
    
    explored_cost = nodes_explored_discrete() # create a dictionary for every node in the workspace, they're all assigned a cost of zero initially
    explored_cost[start_pos] = 0 # assign the starting node a c2c of 0
    
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
            
            if current_node is None or cost[pos_index] + H*1.35 < cost_current: # i dont get this?
                
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
            # print(" explored_cost: ", explored_cost)
            print(" Goal is Reached", iterations)
            # print(" type(current_child_nodes): ", type(current_child_nodes), "\n current_child_nodes: ", current_child_nodes)
            # print(" type(explored_cost): ", type(explored_cost), "len(explored_cost)", len(explored_cost)) # \n explored_cost: ", explored_cost)
            # print("closed_list: ", closed_list)
            # print("\nstore_OL: ", store_OL)
            # print("\nopen_list: ", open_list)
            # print("\nclosed_list: ", closed_list)
            # print("\nexplored_cost: ", explored_cost)

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
            cost[c_indx] = cost2come[c_indx] + cost2go_updated # F = G + H
            Childs_n_Parent.append((current_node, new_pos_node))

def random_sample(clearance, radius):

    state = False

    while state == False:
        randomX = randint(0, width)
        randomY = randint(0, height)
        # robot_traj_coor(float(parentNode[0], float(parentNode[1]), float(parentNode[2]), ))
        randomNode = (randomX, randomY, randint(0, 360), 0, 0, 0)
        state = newValidNodeChecker(randomNode, clearance, radius)
    # print("randomNode: ", randomNode)
    
    return randomNode

def find_closest_neighbors(explored_cost, current_node, threshold):
    # explored_cost is a dictionary
    
    nodesWithinDistance = {}
    x1 = current_node[0]
    y1 = current_node[1]

    # print(" explored_cost: ", explored_cost)

    for key in explored_cost.keys():
        x2, y2, _ = key
        distance = euclidean_dist(x1, y1, x2, y2)

        if distance < threshold:
            c2c = explored_cost[key]
            # nodesWithinDistance.append((round(x2, 2), round(y2, 2), _))
            nodesWithinDistance[round(x2, 2), round(y2, 2)] = c2c

    # print(" distance threshold is: ", threshold, " nodesWithinDistance format: (x, y, theta): c2c", "\n nodesWithinDistance: ", nodesWithinDistance)
    # time.sleep(5)
    return nodesWithinDistance

def choose_parent(current_node, nodesWithinDistance): # the best parent for a new sample within a defined radius is chosen by finding the lowest C2C
    # need positions of nodeWithinDistance, their parents/index and C2C
    best_parent = min(nodesWithinDistance, nodesWithinDistance.get()) # find the node in the dictionary with the lowest C2C

    return best_parent, current_node

# tree = {}
# def rrt(parent_node, neighbor_nodes):

#     for neighbor in neighbor_nodes:
#         tree[neighbor] = parent_node

#     return
        
 
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
    # print("x_coords: ", x_coords)
    # print("y_coords: ", y_coords)

    return x_coords, y_coords




def Curved_Line_Robot(nodePar, nodeCh, linecolor):
    
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
        
        plt.plot([xs, xn],[ys, yn], color=linecolor, linewidth=0.6)



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
    
    print(" The Simulation has Started")
    # print(" nodes: ", nodes)
    
    for node in nodes:
        
        Curved_Line_Robot(node[0], node[1], "orange")
        mapImg = bufImage()


        if cv2.waitKey(1) == ord('q'):
            
            exit()
        cv2.imshow('Robot - A Star', mapImg)

    plt.plot(xCoords, yCoords, c='blue', marker = '.')
    mapImg = bufImage()
    cv2.imshow('Robot - A Star', mapImg)



            
goal_radius = 0.5

start, goal, RPM1, RPM2, R, map_c = user_goals()

start = tuple(start)
goal = tuple(goal)


        
duplicate_nodes = nodes_visited() # initialize multidimensional array with a value of zero, this will be used to check for duplicate nodes
        
visited_nodes, node_goal_bnds, nodes_path = Robot_ASTAR(start, goal, goal_radius, duplicate_nodes, R, map_c, RPM1, RPM2)
# i think we can use visited_nodes / current_child_nodes to backtrack
xCoords, yCoords = backtrack(visited_nodes)


# print("nodes_path: ", nodes_path)
Obs_space = ObsMap(map_c, R)                                        # Creating an instance of the Obstacle Space Object 
goal_circ = plt.Circle((goal[0],goal[1]), radius=goal_radius, color='#F0DB4F')    # Drawing a goal threshold area in the map
Obs_space.ax.add_patch(goal_circ)                            # Adding goal circle drawing to map
goal_circ = plt.Circle((start[0],start[1]), radius=0.1, color='#333399')    # Drawing a goal threshold area in the map
Obs_space.ax.add_patch(goal_circ)   


Simulated_BotShow(nodes_path, xCoords, yCoords)


if cv2.waitKey(0):
    
    exit()
    
cv2.destroyAllWindows()
                            
               
