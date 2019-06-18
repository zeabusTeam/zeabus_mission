#! /usr/bin/python2

from control_lib import Control
import math
import rospy


def main():
    rospy.init_node('Test_Control_lib')
    control = Control('Test')
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
    rospy.loginfo("Robot will forward 2 meters.")
    control.moveDist([2, 0, 0, 0, 0, 0])
    control_wait(control)
    rospy.loginfo("Robot will go deep 2 meters.")
    control.moveDist([0, 0, -2, 0, 0, 0])
    control_wait(control)
    rospy.loginfo("Robot will go left 2 meters.")
    control.moveDist([0, 2, 0, 0, 0, 0])
    control_wait(control)
    rospy.loginfo("Robot will turn 90 degree cww.")
    control.moveDist([0, 0, 0, 0, 0, 90])
    control_wait(control)
    rospy.loginfo("Robot will go up 2 meters.")
    control.moveDist([0, 0, 2, 0, 0, 0])
    control_wait(control)


def control_wait(control):
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if control.isOkay():
            break
        r.sleep()
    return True


if __name__ == "__main__":
    main()
