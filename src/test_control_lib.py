#! /usr/bin/python2

from control_lib import Control
import math


def main():
    control = Control('Test', True)
    print(control.calcNewPosition(
        [0, 0, 0, 0, 0, math.pi/4], [1, -1, 0, 0, 0, 0]))
    print(control.calcNewPosition(
        [0, 0, 0, 0, 0, 3*math.pi/4], [1, -1, 0, 0, 0, 0]))
    print(control.calcNewPosition(
        [0, 0, 0, 0, 0, 5*math.pi/4], [1, -1, 0, 0, 0, 0]))
    print(control.calcNewPosition(
        [0, 0, 0, 0, 0, -math.pi/4], [1, -1, 0, 0, 0, 0]))
    print(control.calcNewPosition(
        [0, 0, 0, 0, 0, math.pi/6], [3, 2, 0, 0, 0, 0]))


if __name__ == "__main__":
    main()
