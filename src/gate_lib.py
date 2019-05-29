import rospy
# w8ing for control command


class Gate:

    def __init__(self, gate_proxy):
        self.gate_proxy = gate_proxy
        self.param = {
            'firstFinding': {
                'threshold': 0.5,
                'rotateAngle': 3,
                'maxAngle': 360  # Should be ~120 when switch is used.
            },
            'forwardToGate': {
                'cxThresold': 0.5,
                'rotateAngle': 4,
                'moveDist': 0.2,
            },
            'finalMoveDist': 2.0,
            'endThreshold': 0.1,
            'visionStatusHistory': 20,
        }

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
            if vision_resp.found == 1:
                self.setGateStatus(1)
            if self.getGateStatus() >= self.param['firstFinding']['threshold']:
                return True
            # Code when switch is available
            # if rotate_count > 90:
            if rotate_count > self.param['firstFinding']['maxAngle']:
                return False
            # rotate command ccw self.param['firstFinding']['rotateAngle'] deg
            rotate_count += self.param['firstFinding']['rotateAngle']
            rospy.sleep(1/10)

    def step02_forwardWithRotate(self):
        """
            if center x is negative and less than ... (might be -0.5) and not in turning phase:
                turn left until ... (might be -0.1)
            elif center x is positive and more than ... (might be 0.5) and not in turning phase:
                turn right until ... (might be 0.1)
            continue_forward_command
        """
        while not rospy.is_shutdown():
            vision_resp = self.gate_proxy()
            if vision_resp.found == 1:
                self.setGateStatus(1)
            if vision_resp.cx1 < -self.param['forwardToGate']['cxThresold']:
                # rotate command ccw self.param['forwardToGate']['rotateAngle'] deg
                pass
            elif vision_resp.cx1 > self.param['forwardToGate']['cxThresold']:
                # rotate command cw self.param['forwardToGate']['rotateAngle'] deg
                pass
            else:
                # move forward
                pass
            rospy.sleep(1/10)
            if self.isEnd():
                return True

    def step02_forwardWithMoveLeftRight(self):
        """
            if center x is negative and less than ... (might be -0.5) and not in moving phase:
                move left until ... (might be -0.1)
            elif center x is positive and more than ... (might be 0.5) and not in moving phase:
                move right until ... (might be 0.1)
            continue_forward_command
        """
        while not rospy.is_shutdown():
            vision_resp = self.gate_proxy()
            if vision_resp.found == 1:
                self.setGateStatus(1)
            if vision_resp.cx1 < -self.param['forwardToGate']['cxThresold']:
                # move left self.param['forwardToGate']['moveDist'] m
                pass
            elif vision_resp.cx1 > self.param['forwardToGate']['cxThresold']:
                # move right self.param['forwardToGate']['moveDist'] m
                pass
            else:
                # move forward
                pass
            rospy.sleep(1/10)
            if self.isEnd():
                return True

    def step03_moveForward(self):
        # forward command with fixed distrace
        pass
