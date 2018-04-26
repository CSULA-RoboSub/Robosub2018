import math


def print_confusion_matrix(result, labels):
    result0 = [int(x) for x in result]

    falsePositives = 0
    falseNegatives = 0
    truePositives = 0
    trueNegatives = 0

    for i, row in enumerate(labels):
        if row == 1 and result[i] == 1:
            truePositives += 1
        if row != 1 and result[i] != 1:
            trueNegatives += 1
        if row != 1 and result[i] == 1:
            falsePositives += 1
        if row == 1 and result[i] != 1:
            falseNegatives += 1

    print "true pos:", truePositives, "true neg:", trueNegatives, "false pos:", falsePositives, "false neg:", falseNegatives, "\n"


def find_angle(center, x, y, w, h):
    midpoint = (x + (w / 2), y + (h / 2))
    angle = math.atan2(midpoint[1] - center[1], midpoint[0] - center[0])
    return angle


colors = {"green": (0, 255, 0), "black": (0, 0, 0), "magenta": (255, 0, 255),  "blue": (255, 0, 0),  "gold": (255, 165, 0), "white": (255, 255, 255), "red": (0, 0, 255)}


def get_directions(center, x, y, w, h):
    directions = [0,0]
    w_pad = w / 3
    h_pad = h / 3
    cx = center[0]
    cy = center[1]
    if cx < x + w_pad:
        if cx > x + (2 * w_pad):
            directions[0] = 0
        else:
            directions[0] = 1
    else:
        directions[0] = -1
    if cy > y + h_pad:
        if cy < y + (2 * h_pad):
            directions[1] = 0
        else:
            directions[1] = 1
    else:
        directions[1] = -1
    return directions


def center(ob):
    return (ob[0] + ob[2] / 2), (ob[1] + ob[3] / 2)


def dist(pt):
    return math.sqrt((pt[1][0] - pt[0][0])**2 + (pt[1][1] - pt[0][1])**2)