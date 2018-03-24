import numpy as np
import random

beacons = np.array([             # nodes of the walls
    [0, 0],
    [0, 200],
    [200, 200],
    [200, 0],
    [0, 100],
    [50, 100]
])

walls = np.array([                               # from which node to which we draw walls
    [beacons[0], beacons[1]],
    [beacons[1], beacons[2]],
    [beacons[2], beacons[3]],
    [beacons[3], beacons[0]],
    [beacons[4], beacons[5]]
])

pi = np.array([10, 30, 90])      # this is the initial position of the robot (x, y, teta)

print("beacons: ", beacons, "\n walls: ", walls)
print("initial position of the robot: ", pi)

botsize = 10
velocity = 10

while 1:
    movement = np.array([
        velocity * np.cos(np.deg2rad(pi[2])),       # x changes according to the cos(teta)
        velocity * np.sin(np.deg2rad(pi[2])),       # y changes according to the sin(teta)
        0                                           # teta does not change
    ])
    po = movement + pi  # output position
    print("\n----------------------wants to move to: ", po)

    dist_min = 100000       # min distance from the bot to the nearest wall
    distance = 100001       # min distance from the robot to each wall
    collision = np.zeros(2)-1000000         # collision point
    crob = np.tan(np.deg2rad(pi[2])) * pi[0] - pi[1]        # for the robot movement: y = x * tg(teta) +crob

    for w in range(0, walls.shape[0]):      # for each wall
        a1 = walls[w][1] - walls[w][0]          # wall vector
        crush = np.zeros(2) - 1000000           # intersection between the wall and the trajectory
        if a1[0] == 0:              # vertical wall!!!
            if po[2] != 90 and po[2] != 270:    # not vertical movement!!!
                distance = abs(walls[w][1][0] - po[0])
                crush[0] = walls[w][1][0]
                if po[2] == 0 or po[2] == 180:
                    crush[1] = po[1]
                else:
                    crush[1] = (np.cos(np.deg2rad(pi[2])) * walls[w][1][0] + crob) / np.sin(np.deg2rad(pi[2]))
                # print("vertical line. distance = ", distance, "from point: ", crush)

        else:
            m = a1[1]/a1[0]
            c = m * walls[w][1][0] - walls[w][1][1]     # for the wall: -mx +y +c = 0
            distance = abs(-m * po[0] + po[1] + c) / (np.sqrt(m ** 2 + 1))
            # print("\nDistance: ", distance)
            if po[2] != 90 and po[2] != 270:
                crush[0] = (c + crob) / (m - np.tan(np.deg2rad(pi[2])))
            else:
                crush[0] = pi[0]
            crush[1] = (m * crob + np.tan(np.deg2rad(pi[2])) * c) / (m - np.tan(np.deg2rad(pi[2])))

        if distance < dist_min and \
                (walls[w][0][0] <= crush[0] <= walls[w][1][0] or walls[w][1][0] <= crush[0] <= walls[w][0][0]) and \
                (walls[w][0][1] <= crush[1] <= walls[w][1][1] or walls[w][1][1] <= crush[1] <= walls[w][0][1]):
                                        # new minimal distance and the crush is located in the wall!!!!!
            dist_min = distance
            collision = crush

    print("min distance to walls = ", dist_min, "in point: ", collision)

    if dist_min < botsize:      # collision!!!!!!!
        print("\n\ncollision in ", collision, "when trying to move to ", po)
        if 90 > pi[2] > 270:        # recalculate x from the collision point and subtract the robotsize
            po[0] = collision[0] - np.cos(np.deg2rad(pi[2])) * botsize
        else:
            po[0] = collision[0] + np.cos(np.deg2rad(pi[2])) * botsize

        if 0 < pi[2] < 180:         # recalculate y from the collision point and subtract the robotsize
            po[1] = collision[1] - np.sin(np.deg2rad(pi[2])) * botsize
        else:
            po[1] = collision[1] + np.sin(np.deg2rad(pi[2])) * botsize

        print("NEW POSITION: ", po)

        pi = po
        pi[2] = random.randrange(0, 360)

        print("New direction: ", pi[2])
        break
    else:
        pi = po
