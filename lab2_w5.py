from motor_control.AROMotorControl import AROMotorControl
from lab1_w4 import PController, goTo
import numpy as np
from math import sqrt, cos, sin


mc = AROMotorControl()

mc.setZero(1)
mc.setZero(2)


# System size
l1 = 0.06
l2 = 0.165
r1 = 0.060
r2 = 0.163
d = 0.150 / 2


def fg(q1, q2, positive=True):
    return np.array([l1*cos(q1) + l2*cos(q1 + q2), l1*sin(q1) + l2*sin(q1 + q2)])