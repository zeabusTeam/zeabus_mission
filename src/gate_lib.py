from __future__ import division
import rospy
import time
import math
from zeabus.control.command_interfaces import CommandInterfaces


def mean(data):
    """Return the sample arithmetic mean of data."""
    n = len(data)
    if n < 1:
        raise ValueError('mean requires at least one data point')
    return sum(data)/n  # in Python 2 use sum(data)/float(n)


def _ss(data):
    """Return sum of square deviations of sequence data."""
    c = mean(data)
    ss = sum((x-c)**2 for x in data)
    return ss


def stddev(data, ddof=0):
    """Calculates the population standard deviation
    by default; specify ddof=1 to compute the sample
    standard deviation."""
    n = len(data)
    if n < 2:
        raise ValueError('variance requires at least two data points')
    ss = _ss(data)
    pvar = ss/(n-ddof)
    return pvar**0.5


class Gate:

    def __init__(self, gate_proxy):
        self.gate_proxy = gate_proxy
        self.param = {
            'checkDeep': {
                'wanted': -0.7,
                'acceptableError': 0.10,
            },
            'firstFinding': {
                'threshold': 0.7,
                'rotateAngle': 10*3.1416/180,
                # Should be ~120 when switch is used.
                'maxAngle': 120*3.1416/180,
            },
            'forwardToGate': {
                'cxThresold': 0.2,
                'rotateAngle': 10,
                'moveDist': 0.25,
                'normalDist': 0.5,
                'timeLimit': 30,
            },
            'finalMoveDist': 3.0,
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
        self.control.reset_state()
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
                self.gateYaw = self.control.current_pose[5]
                rospy.loginfo('[LookToTheLeftLookToTheRight] Found gate.')
                return True
            # Code when switch is available
            # if rotate_count > 90:
            if rotate_count > self.param['firstFinding']['maxAngle']:
                self.control.reset_state()
                rospy.logwarn('Reach maximum finding angle')
                return False
            # rotate command ccw self.param['firstFinding']['rotateAngle'] deg
            if self.control.check_yaw(0.0873):
                self.control.relative_yaw(
                    self.param['firstFinding']['rotateAngle'])
                rotate_count += self.param['firstFinding']['rotateAngle']
            r.sleep()
        self.control.reset_state()
        return True

    def step01_5_lockYawToGate(self):
        self.control.absolute_yaw(self.gateYaw)
        yaws = [-100 for _ in range(40)]
        r = rospy.Rate(10)
        start = time.time()
        rospy.loginfo('Waiting yaw to stable.')
        while not rospy.is_shutdown():
            yaws.append(self.control.current_pose[5])
            del yaws[0]
            if stddev(yaws) < 5*3.1416/180 and (max(yaws)-min(yaws) < 10):
                break
            if time.time()-start > 5:
                break
            r.sleep()
        rospy.loginfo('Yaw is stable.')
        widths = []
        cxs = []
        rospy.loginfo('Trying to locate the gate.')
        while not rospy.is_shutdown():
            vsResp = self.gate_proxy()
            cxs.append(vsResp.cx1)
            widths.append(abs(vsResp.x_left - vsResp.x_right))
            if len(cxs) > 50:
                break
            r.sleep()
        rospy.loginfo('Gate is located.')
        rospy.loginfo('Adjusting heading.')
        self.gateDist = self.estimateDist(sum(widths)/len(widths))
        gateAngle = self.estimateAngle(sum(cxs)/len(cxs), self.gateDist)
        self.gateYaw += gateAngle
        self.control.absolute_yaw(self.gateYaw)

    def estimateDist(self, width):
        realGateW = 3.048  # meters
        return realGateW/width/2

    def estimateAngle(self, cx, dist):
        return math.atan(cx/dist)

    def step02_forwardWithMoveLeftRight(self):
        """
            if center x is negative and less than ... (might be -0.5) and not in moving phase:
                move left until ... (might be -0.1)
            elif center x is positive and more than ... (might be 0.5) and not in moving phase:
                move right until ... (might be 0.1)
            continue_forward_command
        """
        yaws = [-100 for _ in range(40)]
        start = time.time()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            vsResp = self.gate_proxy()
            yaws.append(self.control.current_pose[5])
            del yaws[0]
            if stddev(yaws) > 5*3.1416/180:
                r.sleep()
                continue
            if self.control.check_xy(0.15, 0.15) and self.control.check_yaw(0.15):
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
                if vsResp.cx1 < -self.param['forwardToGate']['cxThresold']:
                    rospy.loginfo("[GoToGate] Too left")
                    self.control.relative_xy(
                        self.param['forwardToGate']['normalDist'], self.param['forwardToGate']['moveDist'])
                elif vsResp.cx1 > self.param['forwardToGate']['cxThresold']:
                    rospy.loginfo("[GoToGate] Too right")
                    self.control.relative_xy(
                        self.param['forwardToGate']['normalDist'], -self.param['forwardToGate']['moveDist'])
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
