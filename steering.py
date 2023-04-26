# import matplotlib.pyplot as plt
# import numpy as np
# import math

# def cost(Xi,Yi,Thetai,UL,UR):
#     t = 0
#     r = 0.066
#     L = 0.178
#     dt = 0.1
#     Xn=Xi
#     Yn=Yi
#     Thetan = 3.14 * Thetai / 180


# # Xi, Yi,Thetai: Input point's coordinates
# # Xs, Ys: Start point coordinates for plot function
# # Xn, Yn, Thetan: End point coordintes
#     D=0

#     i = 0
#     x = [Xi]
#     y = [Yi]

#     while t<1:
#         t += dt
#         # Xs = Xn
#         # Ys = Yn
#         Delta_Xn = 0.5*r * (UL + UR) * math.cos(Thetan) * dt
#         Delta_Yn = 0.5*r * (UL + UR) * math.sin(Thetan) * dt
#         Xn += Delta_Xn # update the positions each iteration
#         Yn += Delta_Yn
#         x.append(Xn)
#         y.append(Yn)
#         Thetan += (r / L) * (UR - UL) * dt
#         D += math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2) + math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
#         i = i + 1
#     Thetan = 180 * (Thetan) / 3.14
    

#     return Xn, Yn, Thetan, D, x, y

# rpm1 = 2
# rpm2 = 5
# actions = [[0, rpm1], [rpm1, 0], [rpm1, rpm1], [rpm2, rpm2],[0, rpm2], [rpm2, 0],  [rpm1, rpm2], [rpm2, rpm1]]
        
# x_coords = []
# y_coords = []
# for action in actions:
#     k=cost(0,0,45, action[0],action[1])      # (0,0,45) hypothetical start configuration, this dosn't matter for calucating the edges'costs
#     print((k[0], k[1]))
#     print("Distance: ", k[3], "\n")
#     x_coords.append(k[0])
#     y_coords.append(k[1])

# k1 = cost(0, 0, 45, 0, rpm1)
# plt.plot
# k2 = cost(0, 0, 45, rpm1, 0)
# k3 = cost(0, 0, 45, rpm1, rpm1)
# k4 = cost(0, 0, 45, rpm2, rpm2)
# k5 = cost(0, 0, 45, 0, rpm2)
# k6 = cost(0, 0, 45, rpm2, 0)
# k7 = cost(0, 0, 45, rpm1, rpm2)
# k8 = cost(0, 0, 45, rpm2, rpm1)

# plt.title("Path Taken")
# plt.plot(k1[4], k1[5], 'd', label = '[0, rpm1]')
# plt.plot(k2[4], k2[5], 'H', label = '[rpm1, 0]')
# plt.plot(k3[4], k3[5], 'p', label = '[rpm1, rpm1]')
# plt.plot(k4[4], k4[5], '*', label = '[rpm2, rpm2]')
# plt.plot(k5[4], k5[5], 'o', label = '[0, rpm2]')
# plt.plot(k6[4], k6[5], 'x', label = '[rpm2, 0]')
# plt.plot(k7[4], k7[5], 'v', label = '[rpm1, rpm2]')
# plt.plot(k8[4], k8[5], 's', label = '[rpm2, rpm1]')

# plt.legend()


# fig = plt.figure()
# plt.plot(x_coords, y_coords, 'x')
# plt.legend()
# plt.title("Final Positions")
# plt.show()

import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt


def euclidean_dist(xi, yi, xf, yf):
    
    e_dist = math.sqrt((xf - xi)**2 + (yf - yi)**2)
    
    return e_dist

def cost(tree, near_node, current_node, UL, UR):
    t = 0
    r = 0.066
    L = 0.178
    dt = 0.1
    Xn=current_node[0]
    Yn=current_node[1]
    Thetai = current_node[2] # double check
    Thetan = 3.14 * Thetai / 180


# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes
    D=0

    i = 0
    x = [current_node[0]]
    y = [current_node[1]]

    while t<1:
        t += dt
        # Xs = Xn
        # Ys = Yn
        Delta_Xn = 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Delta_Yn = 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Xn += Delta_Xn # update the positions each iteration
        Yn += Delta_Yn
        x.append(Xn)
        y.append(Yn)
        Thetan += (r / L) * (UR - UL) * dt
        D += math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2) + math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
        i = i + 1
    Thetan = 180 * (Thetan) / 3.14



    return Xn, Yn, Thetan, D, x, y

def polynomial(x, *coeffs):
    y = []
    for xx in x:
        y.append(sum([coeffs[i] * xx**i for i in range(len(coeffs))]))
    return y

def steering(rpmLeft, rpmRight, x_data, y_data, current_node, near_node):
    print("\n*******steering function*******")
    print("current_node: ", current_node)
    print("near_node: ", near_node)

    if rpmLeft/rpmRight < 2/3.3 or rpmLeft/rpmRight > 2.25:
        print("rpmLeft/rpmRight: ", rpmLeft/rpmRight, " < .606 or > 2.25")
        return False
    
    angle_radians = np.radians(current_node[2])
    # create a horizontal line at the given angle to be able to flip x_data and y_data over this line
    # this flipped data will be used to create a lower bound to check if the current_node is above it
    slope = math.tan(angle_radians)
    y_int = y_data[0] - slope * x_data[0]
    x_line = np.linspace(np.min(x_data), np.max(x_data), len(x_data))
    y_line = slope * x_line + y_int

    # x_data_flipped = (x_data + (y_data - 2*y_line) / (slope**2 + 1))
    x_data_flipped = []
    y_data_flipped = []
    for i in range(len(x_data)):
        x_flipped = (x_data[i] + (y_data[i] - 2*y_int - 2*slope*x_data[i]) / (slope**2 + 1))
        y_flipped = (2*y_int + 2*slope*x_data[i] - y_data[i])
        x_data_flipped.append(x_flipped)
        y_data_flipped.append(y_flipped)

    # y_data_flipped = (y_line + slope*(x_data + (y_data - 2*y_line) / (slope**2 + 1)))
    # y_data_flipped = (2*y_line - y_data)
    print("y_line[-1]: ", y_line[-1], "\ny_data[-1]: ", y_data[-1], "\ny_data_flipped[-1]: ", y_data_flipped[-1])
    print("distance from y_line[-1] to y_data[-1]: ", euclidean_dist(y_line[-1], x_line[-1], y_data[-1], x_data[-1]))
    print("distance from y_line[-1] to y_data_flipped[-1]: ", euclidean_dist(y_line[-1], x_line[-1], y_data_flipped[-1], x_data[-1]))
    plt.plot(x_data, y_data, 'o', label = 'Original Data')
    plt.plot(x_data, y_data_flipped, 'o', label = "flipped data")
    plt.plot(x_line, y_line, label = 'line at the angle')
    plt.legend()
    plt.show()
    
    p0 = [1, 1, 1] # used provide initial guess for parameters    
    coeffs, _ = curve_fit(polynomial, x_data, y_data, p0)
    flipped_coeffs, _ = curve_fit(polynomial, x_data, y_data_flipped, p0)
    print("coeffs: ", coeffs, "\nequation is: ", coeffs[0], coeffs[1], "x +", coeffs[2], "x**2" )
    print("flipped_coeffs: ", flipped_coeffs, "\nequation is: ", flipped_coeffs[0], flipped_coeffs[1], "x +", flipped_coeffs[2], "x**2" )

    # print("1st condition: ", current_node[1] <= coeffs[0] + coeffs[1] ** current_node[0] + coeffs[2] * current_node[0] ** 2)
    # print(coeffs[0] + coeffs[1] ** current_node[0] + coeffs[2] * current_node[0] ** 2)
    # print("2nd condition: ", current_node[0] >= x_data[0] and current_node[0] <= x_data[-1])



    # the following checks if near_node is within the workspace of the mobile robot, we have to check this bc its a differential drive

    # quadrants 1 and 4
    if current_node[2] < 90 or current_node[2] > 270:
        # if y <= c1 + c2x + c3x^2 and x_data[0] <= x <= x_data[-1]
        if near_node[1] <= coeffs[0] + coeffs[1] * near_node[0] + coeffs[2] * near_node[0] ** 2 \
            and near_node [1] >= flipped_coeffs[0] + flipped_coeffs[1] * near_node[0] + flipped_coeffs[2] * near_node[0]**2 \
            and near_node[0] >= x_data[0] and near_node[0] <= x_data[-1]:
            print("\nnear_node", near_node, "is in bounds")
            return True
        else:
            print("near_node", near_node, "is out of bounds")
            return False
    
    # quadrants 2 and 3
    elif current_node[2] > 90:
        print("\nelif")
        # if y <= c1 + c2x + c3x^2 and x_data[0] <= x <= x_data[-1]
        # print("ans: ", coeffs[0] + coeffs[1] * near_node[0] + coeffs[2] * near_node[0] ** 2)
        # print("ans2: ", flipped_coeffs[0] + flipped_coeffs[1] * near_node[0] + flipped_coeffs[2] * near_node[0]**2)
        # print("x")
        if near_node[1] >= coeffs[0] + coeffs[1] * near_node[0] + coeffs[2] * near_node[0] ** 2 \
            and near_node [1] <= flipped_coeffs[0] + flipped_coeffs[1] * near_node[0] + flipped_coeffs[2] * near_node[0]**2 \
            and near_node[0] >= x_data[-1] and near_node[0] <= x_data[0]: ###### issue here
            print("\nnear_node", near_node, "is in bounds")
            return True
        else:
            print("near_node", near_node, "is out of bounds")
            return False








x_coords = []
y_coords = []

tree = {}
nearNode = (6.15, 6.82, 30)
currentNode = (6, 7, 320)
rpmLeft = 2
rpmRight = 3.3
print("rpmLeft/rpmRight: ", rpmLeft/rpmRight, "\n")

actions = [[0, rpmLeft], [rpmLeft, 0], [rpmLeft, rpmLeft], [rpmRight, rpmRight],[0, rpmRight], [rpmRight, 0],  [rpmLeft, rpmRight], [rpmRight, rpmLeft]]


for action in actions:
    k=cost(tree, nearNode, currentNode, rpmLeft, rpmRight)      # (0,0,45) hypothetical start configuration, this dosn't matter for calucating the edges'costs
    # print((k[0], k[1]))
    # print("Distance: ", k[3], "\n")
    x_coords.append(k[0])
    y_coords.append(k[1])

# k1 = cost(tree, nearNode, currentNode, 0, rpmLeft)
# plt.plot
# k2 = cost(tree, nearNode, currentNode, rpmLeft, 0)
k3 = cost(tree, nearNode, currentNode, rpmLeft, rpmRight)
# k4 = cost(tree, nearNode, currentNode, rpmRight, rpmRight)
# k5 = cost(tree, nearNode, currentNode, 0, rpmRight)
# k6 = cost(tree, nearNode, currentNode, rpmRight, 0)
# k7 = cost(tree, nearNode, currentNode, rpmLeft, rpmRight)
# k8 = cost(tree, nearNode, currentNode, rpmRight, rpmLeft)

# plt.title("Path Taken")
# plt.plot(k1[4], k1[5], 'd', label = '[0, 2]')
# plt.plot(k2[4], k2[5], 'H', label = '[rpmLeft, 0]')
plt.plot(k3[4], k3[5], 'p', label = '[rpmLeft, rpmRight]')
print("x_data: ", k3[4], "\ny_data: ", k3[5])
# plt.plot(k4[4], k4[5], '*', label = '[rpmRight, rpmRight]')
# plt.plot(k5[4], k5[5], 'o', label = '[0, rpmRight]')
# plt.plot(k6[4], k6[5], 'x', label = '[rpmRight, 0]')
# plt.plot(k7[4], k7[5], 'v', label = '[rpmLeft, rpmRight]')
# plt.plot(k8[4], k8[5], 's', label = '[rpmRight, rpmLeft]')

plt.legend()
plt.show()

# fig = plt.figure()
# plt.plot(x_coords, y_coords, 'x')
# plt.legend()
# plt.title("Final Positions")
# plt.show()

# currentNode = (5.04, 5.05, 30)
print("currentNode: ", currentNode)
steering(rpmLeft, rpmRight, k3[4], k3[5], currentNode, nearNode)

p0 = [1, 1, 1]
coeffs, _ = curve_fit(polynomial, k3[4], k3[5], p0)
fig = plt.figure()
plt.plot(k3[4], k3[5], 'o', label = 'Data')
plt.plot(k3[4], polynomial(k3[4], *coeffs), label = 'fitted curve')
plt.legend()
plt.title("fitted curve")
plt.show()
