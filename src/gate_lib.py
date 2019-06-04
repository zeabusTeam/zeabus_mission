import rospy
import time
from control_lib import Control


class Gate:

    def __init__(self, gate_proxy):
        self.gate_proxy = gate_proxy
        self.param = {
            'firstFinding': {
                'threshold': 0.8,
                'rotateAngle': 3,
                'maxAngle': 360,  # Should be ~120 when switch is used.
            },
            'forwardToGate': {
                'cxThresold': 0.5,
                'rotateAngle': 4,
                'moveDist': 0.2,
                'normalDist': 0.5,
                'timeLimit': 30,
            },
            'finalMoveDist': 2.0,
            'endThreshold': 0.9,
            'visionStatusHistory': 20,
        }
        self.control = Control('Gate')

    listGateStatus = [0]

    def setGateStatus(self, status=0):
        self.listGateStatus.append(status)
        if len(self.listGateStatus) > self.param['visionStatusHistory']:
            del self.listGateStatus[0]

    def getGateStatus(self):
        return sum(self.listGateStatus)/len(self.listGateStatus)

    def isEnd(self):
        return self.getGateStatus() < self.param['endThreshold']

    def step01_rotateAndFindGate(self):
        """
            TODO: check switch (middle switch = ccw, edge switch = cw)
        """
        rotate_count = 0
        while not rospy.is_shutdown():
            vision_resp = self.gate_proxy()
            # print(vision_resp)
            if vision_resp.found == 1:
                self.setGateStatus(1)
            else:
                self.setGateStatus(0)
            if self.getGateStatus() >= self.param['firstFinding']['threshold']:
                return True
            # Code when switch is available
            # if rotate_count > 90:
            if rotate_count > self.param['firstFinding']['maxAngle']:
                rospy.logwarn('Reach maximum finding angle')
                return False
            # rotate command ccw self.param['firstFinding']['rotateAngle'] deg
            result = self.control.moveDist(
                [0, 0, 0, 0, 0, self.param['firstFinding']['rotateAngle']])
            if result:
                rotate_count += self.param['firstFinding']['rotateAngle']
            else:
                rospy.logerr('Cannot connect to Control system')
                return False
            rospy.sleep(1/10)
        return True

    def step02_forwardWithRotate(self):
        """
            if center x is negative and less than ... (might be -0.5) and not in turning phase:
                turn left until ... (might be -0.1)
            elif center x is positive and more than ... (might be 0.5) and not in turning phase:
                turn right until ... (might be 0.1)
            continue_forward_command
        """
        start = time.time()
        while not rospy.is_shutdown():
            vision_resp = self.gate_proxy()
            if vision_resp.found == 1:
                self.setGateStatus(1)
            else:
                self.setGateStatus(0)
            if self.isEnd() and (time.time()-start > self.param['forwardToGate']['timeLimit']):
                return True
            if vision_resp.cx1 < -self.param['forwardToGate']['cxThresold']:
                result = self.control.moveDist(
                    [0, 0, 0, 0, 0, self.param['forwardToGate']['rotateAngle']])
            elif vision_resp.cx1 > self.param['forwardToGate']['cxThresold']:
                result = self.control.moveDist(
                    [0, 0, 0, 0, 0, -self.param['forwardToGate']['rotateAngle']])
            else:
                result = self.control.moveDist(
                    [self.param['forwardToGate']['normalDist'], 0, 0, 0, 0, 0])
            if not result:
                return False
            rospy.sleep(1/10)

    def step02_forwardWithMoveLeftRight(self):
        """
            if center x is negative and less than ... (might be -0.5) and not in moving phase:
                move left until ... (might be -0.1)
            elif center x is positive and more than ... (might be 0.5) and not in moving phase:
                move right until ... (might be 0.1)
            continue_forward_command
        """
        start = time.time()
        while not rospy.is_shutdown():
            vision_resp = self.gate_proxy()
            if vision_resp.found == 1:
                self.setGateStatus(1)
            else:
                self.setGateStatus(0)
            if self.isEnd() and (time.time()-start > self.param['forwardToGate']['timeLimit']):
                return True
            if vision_resp.cx1 < -self.param['forwardToGate']['cxThresold']:
                result = self.control.moveDist(
                    [0, self.param['forwardToGate']['moveDist'], 0, 0, 0, 0])
            elif vision_resp.cx1 > self.param['forwardToGate']['cxThresold']:
                result = self.control.moveDist(
                    [0, -self.param['forwardToGate']['moveDist'], 0, 0, 0, 0])
            else:
                result = self.control.moveDist(
                    [self.param['forwardToGate']['normalDist'], 0, 0, 0, 0, 0])
            if not result:
                return False
            rospy.sleep(1/10)

    def step03_moveForward(self):
        return self.control.moveDist(
            [self.param['finalMoveDist'], 0, 0, 0, 0, 0])
