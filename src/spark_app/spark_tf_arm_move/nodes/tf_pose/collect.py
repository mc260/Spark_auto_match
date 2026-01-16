import math
import os
import sys

import cv2

def pointDistance(keyPoint):
    """
    :param keyPoint:
    :return:list
    :distance:
    """
    distance0 = (keyPoint[4][0] - keyPoint[9][0]) ** 2 + (keyPoint[4][1] - keyPoint[9][1]) ** 2
    
    distance1 = (keyPoint[7][0] - keyPoint[12][0]) ** 2 + (keyPoint[7][1] - keyPoint[12][1]) ** 2
    
    distance2 = (keyPoint[2][0] - keyPoint[4][0]) ** 2 + (keyPoint[2][1] - keyPoint[4][1]) ** 2
    
    distance3 = (keyPoint[5][0] - keyPoint[7][0]) ** 2 + (keyPoint[5][1] - keyPoint[7][1]) ** 2
    
    distance4 = (keyPoint[0][0] - keyPoint[4][0]) ** 2 + (keyPoint[0][1] - keyPoint[4][1]) ** 2
    
    distance5 = (keyPoint[0][0] - keyPoint[7][0]) ** 2 + (keyPoint[0][1] - keyPoint[7][1]) ** 2
    
    distance6 = (keyPoint[4][0] - keyPoint[10][0]) ** 2 + (keyPoint[4][1] - keyPoint[10][1]) ** 2
    
    distance7 = (keyPoint[7][0] - keyPoint[13][0]) ** 2 + (keyPoint[7][1] - keyPoint[13][1]) ** 2
    
    distance8 = (keyPoint[4][0] - keyPoint[7][0]) ** 2 + (keyPoint[4][1] - keyPoint[7][1]) ** 2
    
    distance9 = (keyPoint[11][0] - keyPoint[14][0]) ** 2 + (keyPoint[11][1] - keyPoint[14][1]) ** 2
    
    distance10 = (keyPoint[10][0] - keyPoint[13][0]) ** 2 + (keyPoint[10][1] - keyPoint[13][1]) ** 2
    
    distance11 = (keyPoint[6][0] - keyPoint[10][0]) ** 2 + (keyPoint[6][1] - keyPoint[10][1]) ** 2
    
    distance12 = (keyPoint[3][0] - keyPoint[13][0]) ** 2 + (keyPoint[3][1] - keyPoint[13][1]) ** 2

    distance13 = (keyPoint[4][0] - keyPoint[11][0]) ** 2 + (keyPoint[4][1] - keyPoint[11][1]) ** 2

    distance14 = (keyPoint[7][0] - keyPoint[14][0]) ** 2 + (keyPoint[7][1] - keyPoint[14][1]) ** 2

    return [distance0, distance1, distance2, distance3, distance4, distance5, distance6, distance7,
            distance8, distance9, distance10, distance11, distance12, distance13, distance14]

def pointAngle(keyPoint):
    angle0  = myAngle(keyPoint[2], keyPoint[3], keyPoint[4])
    angle1  = myAngle(keyPoint[5], keyPoint[6], keyPoint[7])
    angle2  = myAngle(keyPoint[9], keyPoint[10], keyPoint[11])
    angle3  = myAngle(keyPoint[12], keyPoint[13], keyPoint[14])
    angle4  = myAngle(keyPoint[3], keyPoint[2], keyPoint[1])
    angle5  = myAngle(keyPoint[6], keyPoint[5], keyPoint[1])
    angle6  = myAngle(keyPoint[10], keyPoint[8], keyPoint[13])
    angle7  = myAngle(keyPoint[7], keyPoint[12], keyPoint[13])
    angle8  = myAngle(keyPoint[4], keyPoint[9], keyPoint[10])
    angle9  = myAngle(keyPoint[4], keyPoint[0], keyPoint[7])
    angle10 = myAngle(keyPoint[4], keyPoint[8], keyPoint[7])
    angle11 = myAngle(keyPoint[1], keyPoint[8], keyPoint[13])
    angle12 = myAngle(keyPoint[1], keyPoint[8], keyPoint[10])
    angle13 = myAngle(keyPoint[4], keyPoint[1], keyPoint[8])
    angle14 = myAngle(keyPoint[7], keyPoint[1], keyPoint[8])

    return [angle0, angle1, angle2, angle3, angle4, angle5, angle6, angle7,
            angle8, angle9, angle10, angle11, angle12, angle13, angle14]

def myAngle(A, B, C):
    c = math.sqrt((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2)
    a = math.sqrt((B[0] - C[0]) ** 2 + (B[1] - C[1]) ** 2)
    b = math.sqrt((A[0] - C[0]) ** 2 + (A[1] - C[1]) ** 2)
    if a * c != 0:
        return (a ** 2 + c ** 2 - b ** 2) / (2 * a * c)
    return 0

def handAngle(shouder, elbow):
    shouderx = shouder[2] - shouder[0]
    shoudery = shouder[3] - shouder[1]
    elbowx = elbow[2] - elbow[0]
    elbowy = elbow[3] - elbow[1]
    angle1 = math.atan2(shoudery, shouderx)
    angle1 = int(angle1 * 180 / math.pi)
    # print(angle1)
    angle2 = math.atan2(elbowy, elbowx)
    angle2 = int(angle2 * 90 / math.pi)
    # print(angle2)
    if angle1 * angle2 >= 0:
        included_angle = abs(angle1 - angle2)
    else:
        included_angle = abs(angle1) + abs(angle2)
    # included_angle = abs(angle2)
    if included_angle > 90:
        included_angle = 180 -included_angle
    return included_angle
