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
    
    for x in np.arange(0,height,1)/0.6:
        for y in np.arange(0,width,1)/0.6:
            for theta in np.arange(0,120,10)/10:
                
                already_visited[x,y,theta] = 0
                
    return already_visited


def nodes_explored_discrete():
    
    exploredcost = {}
    
    for x in np.arange(0,height,1)/0.6:
        for y in np.arange(0,width,1)/0.6:
            for theta in np.arange(0,120,10)/10:
                
                exploredcost[x,y,theta] = 0
            
    return exploredcost
                
                
def duplicate_checker(current_node, visited_nodes): # use this function everytime we loop through robot Astar to see if there is a duplicate node
    
    duplicate_nodes = False
    
    if current_node in visited_nodes:
        
        if visited_nodes[current_node] == 1:
            
            duplicate_nodes = True
            
        else:
            
            visited_nodes[current_node] = 1
            
            
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
    
    node_goal_bnds = (0,0,0) # the bounds of the node to goal node
    
    

    Childs_n_Parent = []
 
    duplicates[start_pos] = 1
    
    explored_cost = nodes_explored_discrete()
    explored_cost[start_pos] = 0
    
    cost2come = {start_pos:0} # G
    cost2go = {start_pos: Heuristic_Distance(start_pos, goal_pos)} # H
    cost = {start_pos: Heuristic_Distance(start_pos, goal_pos)} # F
    
    
    open_path = (cost[start_pos], start_pos)
    
    open_list = PriorityQueue()
    closed_list = set()
    
    store_OL = set([start_pos])
    exploredPath = {}


    previous_visited = []
    open_list.put(open_path)
    Parent = {} # contains an adjacency map of all nodes

    
    current_child_nodes = {}
    

    
    # Open list created just for Storing the Queue Values
    

    
    iterations = 0
    
    
    
    while len(store_OL) > 0:
        
        current_node = None # node with lowest f() is found
        
        neighbor_nodes = []
 
        
        cost_current = None
        
        
        for node in store_OL:
            
            H = Heuristic_Distance(node, goal_pos)
            pos_index = (node[0], node[1], node[2])
            
            if current_node is None or cost[pos_index] + H*1.35 < cost_current:
                
                cost_current = cost[pos_index]
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
           
            print("Goal is Reached", iterations)
            
            return current_child_nodes, node_goal_bnds, previous_visited


        store_OL.remove(current_node)
        closed_list.add(current_node)
        
        neighbor_nodes = Nodes_per_Action(current_node, RPM1, RPM2) # Robot Moves
        
        cur_index = (current_node[0], current_node[1], current_node[2])

        iterations = iterations + 1
        
        for new_pos_node in neighbor_nodes:
            
            if newValidNodeChecker(new_pos_node, clearance, radius) == False:
                
                continue

    
            if new_pos_node in closed_list:
                
                continue

     
            new_node_created = roundUp(new_pos_node)
            
            cost2come_updated = cost2come[cur_index] + new_pos_node[5]
            
            rnd_newCost = rnd_curCost + new_pos_node[5]
            
            if duplicate_checker(new_node_created, duplicates) == False:
    

                # open_list.put(new_pos_node)
                store_OL.add(new_pos_node)
                
                Childs_n_Parent.append((current_node, new_pos_node))
               
                
            elif cost2come_updated >= explored_cost[new_node_created]:
                
                continue
            
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
    print("randomNode: ", randomNode)
    
    return randomNode

# tree = {}
# def rrt(parent_node, neighbor_nodes):

#     for neighbor in neighbor_nodes:
#         tree[neighbor] = parent_node
        

#     return
        
 
def Path_BackTracking(start, goal_bnds, explored_path):
    
    node_path = []
    goal_set = goal_bnds
    node_path.append(goal_set)
    
    while goal_set != start:
        
        node_path.append(explored_path[goal_set])
        goal_set = explored_path[goal_set]
        
    node_path.reverse()
    
    return node_path



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



def bufImage():
    Obs_space.fig.canvas.draw()
    mapImg = np.frombuffer(Obs_space.fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(Obs_space.fig.canvas.get_width_height()[::-1] + (3,))
    mapImg = cv2.cvtColor(mapImg,cv2.COLOR_RGB2BGR)
    return mapImg



def Simulated_BotShow(nodes):
    
    print("The Simulation has Started")
    
    for node in nodes:
        
        Curved_Line_Robot(node[0], node[1], "orange")
        mapImg = bufImage()
        
        if cv2.waitKey(1) == ord('q'):
            
            exit()
            
        cv2.imshow('Robot - A Star', mapImg)
        
        



goal_radius = 0.5




start, goal, RPM1, RPM2, R, map_c = user_goals()


start = tuple(start)
goal = tuple(goal)


        
duplicate_nodes = nodes_visited()
        
visited_nodes, node_goal_bnds, nodes_path = Robot_ASTAR(start, goal, goal_radius, duplicate_nodes, R, map_c, RPM1, RPM2)

# print(nodes_path)
Obs_space = ObsMap(map_c, R)                                        # Creating an instance of the Obstacle Space Object 
goal_circ = plt.Circle((goal[0],goal[1]), radius=goal_radius, color='#F0DB4F')    # Drawing a goal threshold area in the map
Obs_space.ax.add_patch(goal_circ)                            # Adding goal circle drawing to map
goal_circ = plt.Circle((start[0],start[1]), radius=0.1, color='#333399')    # Drawing a goal threshold area in the map
Obs_space.ax.add_patch(goal_circ)   


Simulated_BotShow(nodes_path)


if cv2.waitKey(0):
    
    exit()
    
cv2.destroyAllWindows()
                            
               
