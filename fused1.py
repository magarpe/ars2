import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
import turtle, random, math

noise = 0.5

tsize = 21
shapelinesize = 2
turtleSpeed = 8
robotSize = 25
turtleHead = 0

# gaussianNoise = 0.0001
# angleNoise = 0.4
m = 100
t = 100

kfr = np.diag((noise, noise, noise))
kfa = np.diag((1, 1, 1))
kfc = np.diag((1, 1, 1))
kfi = np.diag((1, 1, 1))
kfb = np.diag((1, 1, 1))

# process coveritance all digonals are 0.01
kfp = np.diag((noise, noise, noise))
kfq = np.diag((noise, noise, noise))

delta_t = 0.1  # 1sec
mean = 0

alpha1 = noise
alpha2 = noise
alpha3 = noise
alpha4 = 0

landmark_list = [(150, 150), (-150, 150), (-150, -150)]


#############################################################################################################################
def createRobot(position, robotColor):
    global tsize, shapelinesize, robotSize, turtleSpeed, turtlePos, turtleHead
    cir = ((10, 0), (9.51, 3.09), (8.09, 5.88), (5.88, 8.09),
           (3.09, 9.51), (0, 10), (-3.09, 9.51), (-5.88, 8.09),
           (-8.09, 5.88), (-9.51, 3.09), (-10, 0), (-9.51, -3.09),
           (-8.09, -5.88), (-5.88, -8.09), (-3.09, -9.51), (-0.0, -10.0),
           (3.09, -9.51), (5.88, -8.09), (8.09, -5.88), (9.51, -3.09))
    line = ((0, 0), (0, 10))
    s = turtle.Shape("compound")
    s.addcomponent(cir, robotColor, "black")
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
    robotTurtle.showturtle()
    return robotTurtle


def moveRobot(robot, turn, dist):
    global turtlePos, turtleHead
    robot.left(turn)
    robot.forward(dist)
    turtlePos = (robot.xcor(), robot.ycor())
    turtleHead = robot.heading()


def drawBeacons(drawTurtle):
    for b in landmark_list:
        drawTurtle.penup()
        drawTurtle.goto(b)
        drawTurtle.dot(15, "red")


#############################################################################################################################

def simple(b):
    sum = 0.0
    for j in range(12):
        sum += random.uniform(-b, b)
    sum = sum / float(j)
    return sum


def odometry(u_pos, x_pos):
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

    plt.plot(x, y, 'r+')

    return np.array([[x], [y], [theta]])  # np.array([[sigma_rot1], [sigma_rot2], [sigma_trans]])  # ,

#############################################################################################################################


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


#############################################################################################################################


def kfprediction(kfx, kfu, kfz):
    global kfp
    kfx = np.dot(kfa, kfx) + np.dot(kfb, kfu)
    kfp = np.dot(kfa, np.dot(kfp, np.transpose(kfa))) + kfr

    pre_y = inv(np.dot(kfc, np.dot(kfp, np.transpose(kfc))) + kfq)
    kg = np.dot(np.dot(kfp, np.transpose(kfc)), pre_y)
    kfx = kfx + np.dot(kg, kfz - np.dot(kfc, kfx))
    kfp = np.dot((kfi - kg * kfc), kfp)

    return kfx

#############################################################################################################################


def main():
    drawTurtle = turtle.Turtle(visible=False)
    drawTurtle.speed(0)
    drawBeacons(drawTurtle)

    pos = np.array([[0.0], [0.0], [0.0]])

    realrobot = createRobot((pos[0][0], pos[1][0]), "silver")
    shadowrobot = createRobot((pos[0][0], pos[1][0]), "red")

    for i in range(0, 50):
        u = np.array([[10], [0], [10]])

        realpos = np.array([[u[2][0] * math.cos(math.radians(u[0][0]))],
                            [u[2][0] * math.sin(math.radians(u[0][0]))],
                            [0]])
        realpos += pos
        print("real position: ", realpos)
        if i == 0:
            moveRobot(realrobot, u[0][0], u[2][0])
        else:
            moveRobot(realrobot, 0, u[2][0])

        odo = odometry(u, pos)
        print("position from odometry: ", odo)
        beam = triangulation(landmark_list, pos)
        beam[2][0] = odo[2][0]
        print("position for beams: ", beam)

        newpos = kfprediction(pos, odo, beam)

        print("position from kalman filter: ", newpos)

        turn = newpos[2][0] - pos[2][0]
        distance = np.sqrt((pos[0][0] - newpos[0][0])**2 + (pos[1][0] - newpos[1][0])**2)

        moveRobot(shadowrobot, turn, distance)

        pos = newpos


main()
