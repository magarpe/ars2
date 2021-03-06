import numpy as np
from numpy.linalg import inv
import turtle, random, math

drawTurtle = None

noise = 2     # error parameter
steps = 60

tsize = 10          # turtle parameters
shapelinesize = 2
turtleSpeed = 1
robotSize = 10.0
turtleHead = 0

velocity = 10

# beacons = np.array([             # nodes of the walls
# [0, 0],
# [0, 150],
# [150, 150],
# [150, 0]
# ])
# walls = np.array([                               # from which node to which we draw walls
#     [beacons[0], beacons[1]],
#     [beacons[1], beacons[2]],
#     [beacons[2], beacons[3]],
#     [beacons[3], beacons[0]]
# ])

kfr = np.diag((noise, noise, noise))                # kalmans filter parameters
kfa = np.diag((1, 1, 1))
kfc = np.diag((1, 1, 1))
kfi = np.diag((1, 1, 1))
kfb = np.diag((1, 1, 1))

# process covariance all diagonals are the noise variable
kfp = np.diag((noise, noise, noise))
kfq = np.diag((noise, noise, noise))

delta_t = 0.1  # 1sec
mean = 0

alpha1 = noise                  # odometry parameters
alpha2 = noise
alpha3 = noise
alpha4 = 0

landmark_list = [(0, 0), (0, 150), (150, 150), (150, 0)]          # beacon positions
beac = 1


#######################################################   GRAPHICS   ###################################################

def createRobot(position, angle, robotColor, shadow=False):
    #
    # CREATES ROBOT SHAPE AND PLACES IN THE INITIAL POSITION
    # INPUTS: initial position (x, y), orientation, color(red for shadow, grey for real)
    # OUTPUTS: created turtle object
    #

    global tsize, shapelinesize, robotSize, turtleSpeed, turtlePos, turtleHead
    s = turtle.Shape("compound")
    cir = ((10, 0), (9.51, 3.09), (8.09, 5.88), (5.88, 8.09),
           (3.09, 9.51), (0, 10), (-3.09, 9.51), (-5.88, 8.09),
           (-8.09, 5.88), (-9.51, 3.09), (-10, 0), (-9.51, -3.09),
           (-8.09, -5.88), (-5.88, -8.09), (-3.09, -9.51), (-0.0, -10.0),
           (3.09, -9.51), (5.88, -8.09), (8.09, -5.88), (9.51, -3.09))
    s.addcomponent(cir, robotColor, "black")
    if not shadow:
        line = ((0, 0), (0, 10))
        s.addcomponent(line, "black", "black")
    turtle.Screen().register_shape(robotColor, s)
    robotTurtle = turtle.Turtle(visible=False)
    robotTurtle.shapesize(outline=shapelinesize)
    robotTurtle.shape(robotColor)
    robotTurtle.speed(turtleSpeed)
    robotTurtle.turtlesize(robotSize / tsize)
    robotTurtle.penup()
    robotTurtle.goto(position)
    if turtleHead is not None:
        robotTurtle.setheading(turtleHead)
    robotTurtle.left(angle)
    robotTurtle.showturtle()
    return robotTurtle


def moveRobotto(robot, point, angle):
    #
    # MOVES ROBOT TO A NEW POSITION AND ANGLES
    # INPUTS: turtle object robot we want to move, new point (x, y) and orientation (degrees)
    # ACTIONS: robot turtle moves to the new position
    #

    global turtlePos, turtleHead
    robot.goto(point)
    robot.left(angle)
    turtlePos = (robot.xcor(), robot.ycor())
    turtleHead = robot.heading()

def drawBeacons():
    #
    # DRAWS THE BEACONS IN THE SCREEN
    # INPUTS: drawing turtle
    # ACTIONS: drawing turtle moves to each beacon position and marks it with a red point
    #
    global drawTurtle
    for b in landmark_list:
        drawTurtle.penup()
        drawTurtle.goto(b)
        drawTurtle.dot(15, "purple")


# def drawWalls():
#     #
#     # DRAWS THE WALLS OF OUR ENVIRONMENT
#     # INPUTS: none
#     # ACTIONS: drawing turtle moves to the each walls first and last point while drawing a line
#     #
#
#     global drawTurtle
#     drawTurtle.speed(0)
#     for i in range(0, walls.shape[0]):
#         if i == 0:
#             drawTurtle.penup()
#         drawTurtle.goto(walls[i][0])
#         drawTurtle.pendown()
#         drawTurtle.goto(walls[i][1])


def drawPath(p, color):
    global drawTurtle
    drawTurtle.penup()
    drawTurtle.goto(p)
    drawTurtle.dot(5, color)


#######################################################   CONTROL   ####################################################


def control(pi):
    #
    # CONTROLLER FOR THE MOVEMENT OF THE ROBOT
    # INPUTS: initial position
    # ACTIONS: robot will drive forwards until it hits a beacon, then will turn to the next
    # OUTPUTS: new position, movement control
    #
    global landmark_list, beac

    # movement = landmark_list[i] - (pi[0], pi[1])
    movement = np.array([[0],
                         [0],
                         [velocity]])

    distance = np.sqrt((landmark_list[beac][0] - pi[0][0])**2 + (landmark_list[beac][1] - pi[1][0])**2)
    teta = pi[2][0]

    if distance < 20:
        movement[0][0] = - 90
        if movement[0][0] < 0:
            movement[0][0] += 360
        beac += 1
        if beac >= len(landmark_list):
            beac = 0

    po = pi + np.array([[velocity * np.cos(np.deg2rad(teta))],
                        [velocity * np.sin(np.deg2rad(teta))],
                        [movement[0][0]]])

    return po, movement

# ###################################################################################  CONTROL option 1 (Marina)
#  def control(pi):
#     #
#     # CONTROLLER FOR THE MOVEMENT OF THE ROBOT AND THE COLLISIONS
#     # INPUTS: initial position
#     # ACTIONS: robot will drive forwards until it hits a wall, then will turn
#     # OUTPUTS: new position, movement control
#     #
#
#     pi = np.array([pi[0][0], pi[1][0], pi[2][0]])
#     movement = np.array([
#         velocity * np.cos(np.deg2rad(pi[2])),  # x changes according to the cos(teta)
#         velocity * np.sin(np.deg2rad(pi[2])),  # y changes according to the sin(teta)
#         0                                      # teta does not change
#     ])
#     po = movement + pi  # output position
#     # print("\n----------------------wants to move to: ", po)
#
#     dist_min = 100000.0  # min distance from the bot to the nearest wall
#     distance = 100001.0  # min distance from the robot to each wall
#     collision = np.zeros(2) - 1000000  # collision point
#     crob = np.tan(np.deg2rad(pi[2])) * pi[0] - pi[1]                # for the robot movement: y = x * tg(teta) - crob
#
#     for w in range(0, walls.shape[0]):  # for each wall
#         a1 = walls[w][1] - walls[w][0]  # wall vector
#         crush = np.zeros(2) - 1000000  # intersection between the wall and the trajectory
#
#         if a1[0] == 0:                                                                      # vertical wall!!!
#             if po[2] != 90 and po[2] != 270:  # not vertical movement!!!
#                 distance = abs(walls[w][1][0] - po[0])
#                 crush[0] = walls[w][1][0]
#                 if po[2] == 0 or po[2] == 180:
#                     crush[1] = po[1]
#                 else:
#                     crush[1] = walls[w][1][0] * np.tan(np.deg2rad(pi[2])) - crob
#                 # print("vertical line. distance = ", distance, "from point: ", crush)
#
#         elif (po[2] == 0 or po[2] == 180) and a1[1] == 0:                           # horizontal wall and movement
#             1
#         else:                                                                             # other cases
#             slope = a1[1] / a1[0]
#             c = -slope * walls[w][1][0] + walls[w][1][1]                                # for the wall: y = mx +c
#             distance = abs(slope * po[0] - po[1] + c) / (np.sqrt(slope ** 2 + 1))
#             # print("\nDistance: ", distance)
#             if po[2] != 90 and po[2] != 270:
#                 crush[0] = (c + crob) / (-slope + np.tan(np.deg2rad(pi[2])))
#             else:
#                 crush[0] = pi[0]
#             crush[1] = (slope * crob + c * np.tan(np.deg2rad(pi[2]))) / (-slope + np.tan(np.deg2rad(pi[2])))
#             # print("horizontal crush", crush)
#
#         if distance < dist_min and \
#                 (walls[w][0][0] <= crush[0] <= walls[w][1][0] or walls[w][1][0] <= crush[0] <= walls[w][0][0]) and \
#                 (walls[w][0][1] <= crush[1] <= walls[w][1][1] or walls[w][1][1] <= crush[1] <= walls[w][0][1]):
#                                                         # new minimal distance and crush is located in the wall!!!!!
#             dist_min = distance
#             collision = crush
#
#     # print("min distance to walls = ", dist_min, "in point: ", collision)
#
#     if dist_min < robotSize:                                                                      # collision!!!!!!!
#         # print("\n\ncollision in ", collision, "when trying to move to ", po)
#         if 90 > pi[2] > 270:  # recalculate x from the collision point and subtract the robotsize
#             po[0] = collision[0] + np.cos(np.deg2rad(pi[2])) * robotSize
#         else:
#             po[0] = collision[0] - np.cos(np.deg2rad(pi[2])) * robotSize
#
#         if 0 < pi[2] < 180:  # recalculate y from the collision point and subtract the robotsize
#             po[1] = collision[1] + np.sin(np.deg2rad(pi[2])) * robotSize
#         else:
#             po[1] = collision[1] - np.sin(np.deg2rad(pi[2])) * robotSize
#
#         # print("NEW POSITION: ", po)
#
#         po[2] = po[2]+150               # new angle after bouncing
#         while po[2] > 360:
#             po[2] -= 360
#         # po[2] = random.uniform(0, 360)
#
#         # print("New direction: ", po[2])
#
#         movement = np.array([[po[2]-pi[2]],             # new movement matrix (angle1, angle2, dsitance)
#                              [0],
#                              [np.sqrt((pi[0] - po[0])**2 + (pi[1] - po[1])**2)]])
#
#     else:                                                                           # no collision
#         movement = np.array([[0],
#                              [0],
#                              [velocity]])
#
#     po = np.array([[float(po[0])], [float(po[1])], [float(po[2])]])
#     return po, movement

# ###########################################################################################  CONTROL option 2 (Nour)
# for i in range(0, 4):
#     print("wall ", i, ": from", walls[i][0], "to ", walls[i][1])
#
# angle = input("please enter the angle of movement: ")
#
# pi = np.array([20, 20, int(angle)])  # this is the inicial position of the robot (x, y, teta)
#
# print("initial position of the robot: ", pi)
#
# botsize = 10
# velocity = 10
#
# movement = np.array([
#     velocity * np.cos(np.deg2rad(pi[2])),  # x changes according to the cos(teta)
#     velocity * np.sin(np.deg2rad(pi[2])),  # y changes according to the sin(teta)
#     0  # teta does not change
# ])
# po = movement + pi  # output position
# # now, we want to make it move straight forward, so this depends of teta (the angle of our robot)
# while (1):
#     if po[0] <= beacons[1][0] or po[0] <= beacons[0][0]:
#         po[0] = botsize / 2
#         print("I've crashed the wall at point", po)
#         break
#     elif po[0] >= beacons[2][0] or po[0] >= beacons[3][0]:
#         po[0] = 100 - (botsize / 2)
#         po[0] = round(po[0], 1)
#         print("I've crashed the wall at point", po)
#         break
#     elif po[1] >= beacons[1][1] or po[1] >= beacons[2][1]:
#         po[1] = 100 - (botsize / 2)
#         po[1] = round(po[1], 1)
#         print("I've crashed the wall at point", po)
#         break
#     elif po[1] <= beacons[0][1] or po[1] <= beacons[3][1]:
#         po[1] = botsize / 2
#         print("I've crashed the wall at point", po)
#         break
#     else:
#         po += movement  # output position
#         po[0] = round(po[0], 1)
#         po[1] = round(po[1], 1)
#
#         print("next position: ", po)

# ###################################################################################  CONTROL version 3 (Lianne)
# def intersectLines(pt1, pt2, ptA, ptB):
#     """ this returns the intersection of Line(pt1,pt2) and Line(ptA,ptB)
#
#         returns a tuple: (xi, yi, valid, r, s), where
#         (xi, yi) is the intersection
#         r is the scalar multiple such that (xi,yi) = pt1 + r*(pt2-pt1)
#         s is the scalar multiple such that (xi,yi) = pt1 + s*(ptB-ptA)
#             valid == 0 if there are 0 or inf. intersections (invalid)
#             valid == 1 if it has a unique intersection ON the segment    """
#     DET_TOLERANCE = 0.00000001
#     # the first line is pt1 + r*(pt2-pt1)
#     # in component form:
#     x1, y1 = pt1;
#     x2, y2 = pt2
#     dx1 = x2 - x1;
#     dy1 = y2 - y1
#     # the second line is ptA + s*(ptB-ptA)
#     x, y = ptA;
#     xB, yB = ptB;
#     dx = xB - x;
#     dy = yB - y;
#     # we need to find the (typically unique) values of r and s
#     # that will satisfy
#     #
#     # (x1, y1) + r(dx1, dy1) = (x, y) + s(dx, dy)
#     #
#     # which is the same as
#     #
#     #    [ dx1  -dx ][ r ] = [ x-x1 ]
#     #    [ dy1  -dy ][ s ] = [ y-y1 ]
#     #
#     # whose solution is
#     #
#     #    [ r ] = _1_  [  -dy   dx ] [ x-x1 ]
#     #    [ s ] = DET  [ -dy1  dx1 ] [ y-y1 ]
#     #
#     # where DET = (-dx1 * dy + dy1 * dx)
#     #
#     # if DET is too small, they're parallel
#     #
#     DET = (-dx1 * dy + dy1 * dx)
#     if math.fabs(DET) < DET_TOLERANCE: return (0, 0, 0, 0, 0)
#     # now, the determinant should be OK
#     DETinv = 1.0 / DET
#     # find the scalar amount along the "self" segment
#     r = DETinv * (-dy * (x - x1) + dx * (y - y1))
#     # find the scalar amount along the input line
#     s = DETinv * (-dy1 * (x - x1) + dx1 * (y - y1))
#     # return the average of the two descriptions
#     xi = (x1 + r * dx1 + x + s * dx) / 2.0
#     yi = (y1 + r * dy1 + y + s * dy) / 2.0
#     return (xi, yi, 1)
#
#
# def IsOnSegment(xi, yi, xj, yj, xk, yk):
#     return (xi <= xk or xj <= xk) and (xk <= xi or xk <= xj) and \
#            (yi <= yk or yj <= yk) and (yk <= yi or yk <= yj)
#
#
# def ComputeDirection(xi, yi, xj, yj, xk, yk):
#     a = (xk - xi) * (yj - yi)
#     b = (xj - xi) * (yk - yi)
#     if a < b:
#         return -1
#     elif a > b:
#         return 1
#     else:
#         return 0
#
#
# # /** Do line segments (x1, y1)--(x2, y2) and (x3, y3)--(x4, y4) intersect? */
# def DoLineSegmentsIntersect(line1, line2):
#     a, b = line1
#     c, d = line2
#     x1, y1 = a
#     x2, y2 = b
#     x3, y3 = c
#     x4, y4 = d
#     d1 = ComputeDirection(x3, y3, x4, y4, x1, y1)
#     d2 = ComputeDirection(x3, y3, x4, y4, x2, y2)
#     d3 = ComputeDirection(x1, y1, x2, y2, x3, y3)
#     d4 = ComputeDirection(x1, y1, x2, y2, x4, y4)
#     return (((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
#             ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0))) or \
#            (d1 == 0 and IsOnSegment(x3, y3, x4, y4, x1, y1)) or \
#            (d2 == 0 and IsOnSegment(x3, y3, x4, y4, x2, y2)) or \
#            (d3 == 0 and IsOnSegment(x1, y1, x2, y2, x3, y3)) or \
#            (d4 == 0 and IsOnSegment(x1, y1, x2, y2, x4, y4))
#
#
# def setNextPosition(target):
#     global robotPos, walls, robotSize, velocity, movement, roomTurtle
#     crashPoint = None
#     crashWall = None
#     # check if any of the walls are in the robot's path
#     for w in walls:
#         c = DoLineSegmentsIntersect((robotPos[0:2], target), w)  # check for passing a wall line
#         if c:
#             # now check if the actual wall is hit, or if the robot comes too close
#             # for the last, create parallel lines to check
#             parallelP = (np.array(w[0]) + robotSize, np.array(w[1]) + robotSize)
#             parallelN = (np.array(w[0]) - robotSize, np.array(w[1]) - robotSize)
#             if DoLineSegmentsIntersect((robotPos[0:2], target), parallelP) or \
#                     DoLineSegmentsIntersect((robotPos[0:2], target), parallelN):
#                 if crashWall is None:
#                     crashWall = w
#                 # see if this wall is closer to the robot than the last found
#                 else:
#                     p1, p2 = crashWall
#                     p3 = robotPos[0:2]
#                     oldWallDistance = norm(np.cross(p2 - p1, p1 - p3)) / norm(p2 - p1)
#                     p1, p2 = w
#                     newWallDistance = norm(np.cross(p2 - p1, p1 - p3)) / norm(p2 - p1)
#                     if oldWallDistance > newWallDistance:
#                         crashWall = w
#     # if a crash point has been found, determine next robot position
#     if crashWall is not None:
#         print("Found a wall to crash into")
#         parallelP = (np.array(crashWall[0]) + robotSize, np.array(crashWall[1]) + robotSize)
#         parallelN = (np.array(crashWall[0]) - robotSize, np.array(crashWall[1]) - robotSize)
#         pointP = intersectLines(robotPos[0:2], target, parallelP[0], parallelP[1])
#         pointN = intersectLines(robotPos[0:2], target, parallelN[0], parallelN[1])
#         # if an intersection has been found, determine distance
#         if pointP[2] == 1:
#             distP = np.linalg.norm(np.array(pointP[0:2]) - np.array(robotPos[0:2]))
#         else:
#             distP = 100000000
#         if pointN[2] == 1:
#             distN = np.linalg.norm(np.array(pointN[0:2]) - np.array(robotPos[0:2]))
#         else:
#             distN = 100000000
#         # find which point is closest to the robot
#         if distP < distN:
#             para = pointP[0:2]
#         elif distP > distN:
#             para = pointN[0:2]
#         print("I crashed at wall", crashWall[0], "to", crashWall[1])
#         print(para)
#         if para is not None:
#             robotPos[0] = para[0]
#             robotPos[1] = para[1]
#
#
# ##        roomTurtle.goto(pointP[0:2])
# ##        roomTurtle.dot(10,"orange")
# ##        roomTurtle.goto(pointN[0:2])
# ##        roomTurtle.dot(10,"orange")
# ##        time.sleep(1)
# moveRobot(para)
#
# # update heading and movement
# robotPos[2] = random.randint(0, 360)
# print("new heading", robotPos[2])
# movement = np.array([
#     velocity * np.cos(np.deg2rad(robotPos[2])),
#     velocity * np.sin(np.deg2rad(robotPos[2])), 0])
# print("New movement:", movement)
# else:  # it didn't find a wall in its way
# moveRobot(target)



######################################################   ODOMETRY   ####################################################


def simple(b):
    #
    # TAKES SAMPLE FROM DISTRIBUTION
    # INPUTS: range of the sample
    # OUTPUTS: sample of the distribution
    #

    sum = 0.0
    for j in range(12):
        sum += random.uniform(-b, b)
    sum = sum / float(j)
    return sum


def odometry(u_pos, x_pos):
    #
    # CALCULATION OF THE CHANGE IN POSITION BASED ON ODOMETRY: motion sensors
    # INPUTS: control of the position, and starting position
    # OUTPUTS: change in position (new control for the kf)
    #

    sigma_rot1 = u_pos[0, 0]
    sigma_rot2 = u_pos[1, 0]
    sigma_trans = u_pos[2, 0]

    theta = x_pos[2, 0]

    sigma_rot1 = sigma_rot1 + simple((alpha1 * abs(sigma_rot1)) + (alpha2 * sigma_trans))
    sigma_trans = sigma_trans + simple(alpha2 * sigma_trans + alpha4 * (abs(sigma_rot1) + abs(sigma_rot2)))
    sigma_rot2 = sigma_rot2 + simple((alpha1 * abs(sigma_rot2)) + alpha2 * sigma_trans)

    x = (sigma_trans * math.cos(math.radians(theta + sigma_rot1)))
    y = (sigma_trans * math.sin(math.radians(theta + sigma_rot1)))
    theta = sigma_rot1 + sigma_rot2

    # plt.plot(x, y, 'r+')

    return np.array([[x], [y], [theta]])

###################################################   TRIANGULATION   ##################################################


def getAngles(beacons):
    global noise, turtlePos
    angles = []
    for b in range(len(beacons)):
        angle = math.degrees(math.atan2(beacons[b][1] - turtlePos[1], beacons[b][0] - turtlePos[0]))
        if angle < 0:
            angle += 360
        if noise is not None:
            angle += np.random.normal(0, noise, 1)
        angles.append(angle)
    return angles


def vlen(vector):
    return math.sqrt(vector[0] * vector[0] + vector[1] * vector[1])


def seglen(pt1, pt2):
    return vlen(psub(pt2, pt1))


def vadd(vec1, vec2):
    return vec1[0] + vec2[0], vec1[1] + vec2[1]


def vscale(scale, vec):
    return scale * vec[0], scale * vec[1]


def psub(pt1, pt2):
    return pt1[0] - pt2[0], pt1[1] - pt2[1]


def pcenter(pt1, pt2):
    v12 = psub(pt2, pt1)  # unit vector from landmark 1 to 2
    return vadd(pt1, vscale(0.5, v12))  # pt1 + 0.5*v12


def vec_eq(vec1, vec2):
    equals = False
    if (vec1[0] == vec2[0]) and (vec1[1] == vec2[1]):
        equals = True
    return equals


def cosine_rule_get_angle(a, b, c):
    return math.acos((a * a + b * b - c * c) / (2 * a * b))


def unitvec(vec):
    len_vec = vlen(vec)
    return vec[0] / len_vec, vec[1] / len_vec


def vdot(vec1, vec2):
    return vec1[0] * vec2[0] + vec1[1] * vec2[1]


def unit_normal(vec, facing_vec):
    if vec[0] == 0:  # ex. (0, 2)
        v_norm = (1, 0)
    elif vec[1] == 0:  # ex. (2, 0)
        v_norm = (0, 1)
    else:
        v_temp = (-1 * vec[1], vec[0])
        v_temp_len = vlen(v_temp)
        v_norm = (v_temp[0] / v_temp_len, v_temp[1] / v_temp_len)
    if vdot(v_norm, facing_vec) >= 0:
        return v_norm
    else:
        return vscale(-1, v_norm)


def v_direction(vec):
    return math.atan2(vec[1], vec[0])


def heading_to_unit_velocity(heading):
    return (math.cos(heading), math.sin(heading))


def properly_order_landmarks(landmark_list, angle_list):
    '''
    Reorders the landmarks as the first step of geometric triangulation.
    '''
    landmark_orders = ((0, 1, 2), (0, 2, 1), (1, 0, 2), (1, 2, 0), (2, 0, 1), (2, 1, 0))
    landmark_order = landmark_orders[0]
    for order_index in range(0, len(landmark_orders)):
        landmark_order = landmark_orders[order_index]
        angle0 = angle_list[landmark_order[0]]
        angle1 = angle_list[landmark_order[1]]
        angle2 = angle_list[landmark_order[2]]
        alpha = (angle1 - angle0) % 360.0
        beta = (angle2 - angle1) % 360.0
        if ((alpha >= 0) and (beta >= 0) and (alpha <= 180) and (beta <= 180)):
            break
    new_angle_list = [0, 0, 0]
    new_landmark_list = [0, 0, 0]
    for order_index in range(0, len(landmark_order)):
        new_angle_list[order_index] = angle_list[landmark_order[order_index]] % 360.0
        new_landmark_list[order_index] = landmark_list[landmark_order[order_index]]
    return new_angle_list, new_landmark_list


def triangulation(landmark_list, eps):
    global angle_list
    angle_list = getAngles(landmark_list)
    (angles, landmarks) = properly_order_landmarks(landmark_list, angle_list)
    alpha = (angles[1] - angles[0]) % 360.0
    alpha = alpha * math.pi / 180
    beta = (angles[2] - angles[1]) % 360.0
    beta = beta * math.pi / 180
    if alpha == 0 and beta == 0:
        print("Significant measurement error (collinear).")
        return
    pt1 = landmarks[0]
    pt2 = landmarks[1]
    pt3 = landmarks[2]
    v12 = unitvec(psub(pt2, pt1))  # unit vector from landmark 1 to 2
    v23 = unitvec(psub(pt3, pt2))  # unit vector from landmark 2 to 3
    d12 = vlen(psub(pt2, pt1))  # distance from point 1 to 2
    d23 = vlen(psub(pt3, pt2))  # distance from 2 to 3
    p12 = pcenter(pt1, pt2)  # pt1 + 0.5*v12
    p23 = pcenter(pt2, pt3)

    if alpha == 0:  # Robot collinear with 1 and 2
        alpha = eps
    if alpha == 180:  # Robot collinear with 1 and 2
        alpha = 180 - eps
    if beta == 0:  # Robot collinear with 2 and 3
        beta = eps
    if beta == 180:  # Robot collinear with 2 and 3
        beta = 180 - eps

    la = 0
    lb = 0
    if not (alpha == 90):
        # if alpha is zero, then la is zero but the tangent blows up
        la = d12 / (2.0 * math.tan(alpha))
    if not (beta == 90):
        lb = d23 / (2.0 * math.tan(beta))
    ra = d12 / (2.0 * math.sin(alpha))  # radius of circle a
    rb = d23 / (2.0 * math.sin(beta))  # radius of circle b

    # ca: center of circle a
    ca = (p12[0] - la * v12[1],
          p12[1] + la * v12[0])
    # cb: center of circle b
    cb = (p23[0] - lb * v23[1],
          p23[1] + lb * v23[0])
    cba = psub(ca, cb)  # points from center of circle b to center of circle a
    if vec_eq(ca, cb):
        print("Significant measurement error (concentric).")
        return

    # get lengths of three segments of triangle (cb pt1 pt2) and find angle cb from the cosine rule
    tri_a = seglen(cb, ca)
    tri_b = seglen(cb, pt2)
    tri_c = seglen(pt2, ca)
    gamma = cosine_rule_get_angle(tri_a, tri_b, tri_c)  # math.asin(vlen(v12)/(2*ra))
    d2r = 2 * rb * math.sin(gamma)
    d2r_vec = vscale(d2r, unit_normal(cba, psub(ca, pt2)))
    # d2r*(the unit normal to cba generally facing from pt2 to ca)
    robot_coord = vadd(pt2, d2r_vec)
    vec_robot_to_pt1 = psub(pt1, robot_coord)
    heading = (v_direction(vec_robot_to_pt1) - (math.pi / 180) * angles[0])
    # unit_velocity = heading_to_unit_velocity(heading)

    return np.array([[robot_coord[0]], [robot_coord[1]], [(180 / math.pi) * heading[0] % 360]])


###############################################   KALMAN FILTER   ######################################################


def kf(kfx, kfu, kfz):
    #
    # CALCULATIONS FOR THE KALMAN FILTER
    # INPUTS: initial position(from odometry), control and position(from triangulation)
    # ACTIONS: robot will drive forwards until it hits a wall, then will turn
    # OUTPUTS: new position, movement control
    #

    global kfp
    # PREDICTION
    kfx = np.dot(kfa, kfx) + np.dot(kfb, kfu)
    kfp = np.dot(kfa, np.dot(kfp, np.transpose(kfa))) + kfr

    # CORRECTION
    pre_y = inv(np.dot(kfc, np.dot(kfp, np.transpose(kfc))) + kfq)
    kg = np.dot(np.dot(kfp, np.transpose(kfc)), pre_y)
    kfx = kfx + np.dot(kg, kfz - np.dot(kfc, kfx))
    kfp = np.dot((kfi - kg * kfc), kfp)

    return kfx

###################################################   MAIN   ###########################################################


def main():

    # INITIALIZATION
    global drawTurtle
    drawTurtle = turtle.Turtle(visible=False)
    drawTurtle.speed(0)
    drawBeacons()
    #drawWalls()

    # initial position
    # pos = np.array([[137.5277675], [138.66025404], [30.0]])
    pos = np.array([[0.0], [0.0], [90.0]])
    realpos = pos
    realposold = realpos

    # creates real and shadow robot
    realrobot = createRobot((realpos[0][0], realpos[1][0]), realpos[2][0], "silver", False)
    shadowrobot = createRobot((pos[0][0], pos[1][0]), pos[2][0], "red", True)

    # u = np.array([[90], [0], [velocity]])
    global steps
    for i in range(steps):
        # ############################################################################  R E A L   R O B O T
        realpos, u = control(realposold)
        # print("u, realpos ", u, realpos)

        print("real position: ", realpos)
        moveRobotto(realrobot, (realpos[0][0], realpos[1][0]), realpos[2][0]-realposold[2][0])
        drawPath(realrobot.pos(), "blue")
        realposold = realpos

        # ############################################################################  S H A D O W   R O B O T

        odo = odometry(u, pos)
        print("position from odometry: ", odo)

        b = random.sample(landmark_list, 3)
        beam = triangulation(b, realpos)
        # beam[2][0] = odo[2][0]
        print("position for beams: ", beam)

        newpos = kf(pos, odo, beam)
        print("position from kalman filter: ", newpos)

        shadowangle = newpos[2][0] - pos[2][0]
        if shadowangle > 360:
            shadowangle -= 360
        if shadowangle < 360:
            shadowangle += 360

        moveRobotto(shadowrobot, (newpos[0][0], newpos[1][0]), 0)
        # moveRobot(shadowrobot, turn, distance)
        drawPath(shadowrobot.pos(), "red")

        pos = newpos


main()
