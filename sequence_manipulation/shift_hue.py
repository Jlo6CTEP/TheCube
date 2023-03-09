from math import sqrt, cos, sin, radians

import numpy as np


def clamp(v):
    if v < 0:
        return 0
    if v > 255:
        return 255
    return int(v + 0.5)


class RGBRotate():
    def __init__(self, degrees):
        self.matrix = np.eye(3)
        cosA = cos(radians(degrees))
        sinA = sin(radians(degrees))
        self.matrix[0][0] = cosA + (1.0 - cosA) / 3.0
        self.matrix[0][1] = 1. / 3. * (1.0 - cosA) - sqrt(1. / 3.) * sinA
        self.matrix[0][2] = 1. / 3. * (1.0 - cosA) + sqrt(1. / 3.) * sinA
        self.matrix[1][0] = 1. / 3. * (1.0 - cosA) + sqrt(1. / 3.) * sinA
        self.matrix[1][1] = cosA + 1. / 3. * (1.0 - cosA)
        self.matrix[1][2] = 1. / 3. * (1.0 - cosA) - sqrt(1. / 3.) * sinA
        self.matrix[2][0] = 1. / 3. * (1.0 - cosA) - sqrt(1. / 3.) * sinA
        self.matrix[2][1] = 1. / 3. * (1.0 - cosA) + sqrt(1. / 3.) * sinA
        self.matrix[2][2] = cosA + 1. / 3. * (1.0 - cosA)

    def apply(self, rgb):
        return np.clip(self.matrix @ rgb, 0, 255)

import cv2

img = cv2.imread('./gaymes.jpg')

shift = RGBRotate(-30)

for x in img:
    for y in x:
        y[:] = shift.apply(y)

cv2.imshow('kek', img)
cv2.waitKey(0)