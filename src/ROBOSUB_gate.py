#!/usr/bin/python2

from __future__ import print_function

import rospy

from zeabus_utility.srv import VisionGate

from gate_lib import Gate


def main():
    rospy.init_node('GateMission')
    rospy.wait_for_service('/vision/gate')
    try:
        gate_srv = rospy.ServiceProxy('/vision/gate', VisionGate)
    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)
    obj = Gate(gate_srv)
    obj.step00_checkDeep()
    obj.step01_rotateAndFindGate()
    obj.step02_forwardWithMoveLeftRight()
    """
    PLAN:
        1. Try to rotate instade of move left/right
            obj.step02_forwardWithRotate()
        2. Move/Rotate while forwarding
    """
    obj.step03_moveForward()  # After move through


if __name__ == "__main__":
    main()
