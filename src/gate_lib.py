import rospy
# w8ing for control command


class Gate:

    def __init__(self, gate_proxy):
        self.gate_proxy = gate_proxy

    listGateStatus = [0]

    def setGateStatus(self, status=0):
        self.listGateStatus.append(status)
        if len(self.listGateStatus) > 20:
            del self.listGateStatus[0]

    def getGateStatus(self):
        return sum(self.listGateStatus)/len(self.listGateStatus)

    def isEnd(self):
        return self.getGateStatus() < 0.1

    def step01_rotateAndFindGate(self):
        """
            TODO: check switch (middle switch = ccw, edge switch = cw)
            if notfound:
                turn left
            elif center x is negative and less than ... (might be -0.5) and not in turning phase:
                turn left until ... (might be -0.1)
            elif center x is positive and more than ... (might be 0.5) and not in turning phase:
                turn right until ... (might be 0.1)

        """
        rotate_count = 0
        while not rospy.is_shutdown():
            vision_resp = self.gate_proxy()
            if vision_resp.found == 1:
                self.setGateStatus(1)
            if self.getGateStatus() >= 0.4:
                return True
            # Code when switch is available
            # if rotate_count > 90:
            if rotate_count > 360:
                return False
            # rotate command ccw 4 deg
            rotate_count += 3
            rospy.sleep(1/10)

    def step02_forwardWithRotate(self):
        """
            TODO: check switch (middle switch = ccw, edge switch = cw)
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
            if vision_resp.cx1 < -0.5:
                # rotate command ccw 4 deg
                pass
            elif vision_resp.cx1 > 0.5:
                # rotate command cw 4 deg
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
            if vision_resp.cx1 < -0.5:
                # move left 0.2m
                pass
            elif vision_resp.cx1 > 0.5:
                # move right 0.2m
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
