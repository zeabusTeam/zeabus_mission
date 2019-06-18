from __future__ import division
import rospy
import time
from zeabus.control.command_interfaces import CommandInterfaces


class Gate:

    def __init__(self, gate_proxy):
        self.gate_proxy = gate_proxy
        self.param = {
            'checkDeep': {
                'wanted': -1.4,
                'acceptableError': 0.10,
            },
            'firstFinding': {
                'threshold': 0.7,
                'rotateAngle': 30,
                'maxAngle': 360,  # Should be ~120 when switch is used.
            },
            'forwardToGate': {
                'cxThresold': 0.2,
                'rotateAngle': 10,
                'moveDist': 0.5,
                'normalDist': 2.0,
                'timeLimit': 30,
            },
            'finalMoveDist': 2.0,
            'endThreshold': 0.2,
            'visionStatusHistory': 20,
        }
        self.control = CommandInterfaces('Gate')

    listGateStatus = [0]

    def setGateStatus(self, status=0):
        self.listGateStatus.append(status)
        if len(self.listGateStatus) > self.param['visionStatusHistory']:
            del self.listGateStatus[0]

    def getGateStatus(self):
        return sum(self.listGateStatus)/len(self.listGateStatus)

    def isEnd(self):
        return self.getGateStatus() < self.param['endThreshold']

    def step00_checkDeep(self):
        rospy.loginfo('Adjusting deep.')
        self.control.absolute_z(-1.2)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.control.check_z(0.15):
                break
            r.sleep()
        rospy.loginfo('AUV Deep is as wanted.')
        return True

    def step01_rotateAndFindGate(self):
        """
            TODO: check switch (middle switch = ccw, edge switch = cw)
        """
        rotate_count = 0
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            vsResp = self.gate_proxy()
            # print(vsResp)
            if vsResp.found == 1:
                self.setGateStatus(1)
            else:
                self.setGateStatus(0)
            if self.getGateStatus() >= self.param['firstFinding']['threshold']:
                self.control.reset_state()
                rospy.loginfo('[LookToTheLeftLookToTheRight] Found gate.')
                return True
            # Code when switch is available
            # if rotate_count > 90:
            if rotate_count > self.param['firstFinding']['maxAngle']:
                self.control.reset_state()
                rospy.logwarn('Reach maximum finding angle')
                return False
            # rotate command ccw self.param['firstFinding']['rotateAngle'] deg
            if self.control.check_yaw(0.15):
                self.control.relative_yaw(
                    self.param['firstFinding']['rotateAngle'])
                rotate_count += self.param['firstFinding']['rotateAngle']
            r.sleep()
        self.control.reset_state()
        return True

    def step02_forwardWithMoveLeftRight(self):
        """
            if center x is negative and less than ... (might be -0.5) and not in moving phase:
                move left until ... (might be -0.1)
            elif center x is positive and more than ... (might be 0.5) and not in moving phase:
                move right until ... (might be 0.1)
            continue_forward_command
        """
        start = time.time()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            vsResp = self.gate_proxy()
            if vsResp.found == 1:
                self.setGateStatus(1)
            else:
                self.setGateStatus(0)
            endCond = (self.isEnd() and
                       (time.time()-start >
                        self.param['forwardToGate']['timeLimit']))
            if endCond:
                self.control.reset_state()
                rospy.loginfo('[GoToGate] Passed gate.')
                return True
            if self.control.check_xy(0.15, 0.15):
                if vsResp.cx1 < -self.param['forwardToGate']['cxThresold']:
                    rospy.loginfo("[GoToGate] Too left")
                    self.control.relative_xy(
                        0, self.param['forwardToGate']['moveDist'])
                elif vsResp.cx1 > self.param['forwardToGate']['cxThresold']:
                    rospy.loginfo("[GoToGate] Too right")
                    self.control.relative_xy(
                        0, -self.param['forwardToGate']['moveDist'])
                else:
                    rospy.loginfo("[GoToGate] Forward")
                    self.control.relative_xy(
                        self.param['forwardToGate']['normalDist'], 0)
            r.sleep()

    def step03_moveForward(self):
        rospy.loginfo(
            '[MoveMore] I need more move to passed the gate all of robot.'
        )
        self.control.relative_xy(
            self.param['finalMoveDist'], 0)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.control.check_xy(0.15, 0.15):
                break
            r.sleep()
        rospy.loginfo('[MoveMore] Passed gate.')
        return True
