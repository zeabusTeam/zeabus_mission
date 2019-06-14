#! /usr/bin/python2
import rospy
from zeabus_utility.srv import SendControlCommand
from zeabus_utility.srv import GetAUVState
from zeabus_utility.msg import ControlCommand
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion
import math


class Control:

    def __init__(self, senderName, debug=False):
        """Control wrapper class for Mission Planner

        Keyword Arguments:
            senderName {str}
                -- Everyone that use this class must to privide your name.
            debug {boolean}
                -- Enable debug mode
        """
        # rospy.init_node('MissionControlLib', anonymous=True)

        self.AuvStateSrvName = '/fusion/auv_state'
        self.CtlCmdSrvName = '/control/interfaces'
        self.seq = 0  # init first sequence no.
        self.senderName = rospy.get_name()+'.'+senderName
        self.targetState = None
        self.DEBUG = debug

        rospy.loginfo('Waiting for %s srv to be available.' %
                      self.CtlCmdSrvName)
        if not self.DEBUG:
            rospy.wait_for_service(self.CtlCmdSrvName)
        else:
            rospy.loginfo('Debug mode is ON. Skip waiting CtlCmd')

        self.proxy = rospy.ServiceProxy(
            self.CtlCmdSrvName, SendControlCommand)
        self.AUVStateProxy = rospy.ServiceProxy(
            self.AuvStateSrvName, GetAUVState)

        self.lastCommand = {
            'type': None,
            'target': None,
            'mask': None
        }

    def moveDist(self, direction=[0, 0, 0, 0, 0, 0], ignoreZero=False):
        """Tell robot to move ... meter(s).

        Keyword Arguments:
            direction {list} -- [x, y, z, roll, pitch, yaw]
                (default: [0,0,0,0,0,0])

        Returns:
            {bool} -- Command sending status

        Note:
            Use right hand rules for angle input.
        """

        # Convert deg to rad
        direction[3:6] = [x*math.pi/180 for x in direction[3:6]]

        currentPos = self.getCurrent()
        newPos = self.calcNewPosition(currentPos, direction)

        mask = []
        for i, x in enumerate(direction):
            if x == 0:
                newPos[i] = 0.0
            mask.append(ignoreZero or (x != 0))

        if self.lastCommand['type'] is not None:
            for i, x in enumerate(direction):
                if x == 0 and self.lastCommand['target'][i] == 0:
                    mask[i] = False

        rospy.logdebug('Old:%s Command:%s New:%s' %
                       (str(currentPos), str(direction), str(newPos)))

        head = Header()
        head.seq = self.seq
        head.stamp = rospy.Time.now()
        head.frame_id = self.senderName

        command = ControlCommand()
        command.header = head
        command.target = newPos
        command.mask = mask

        self.seq += 1

        try:
            if not self.DEBUG:
                self.proxy(command)
        except rospy.ServiceException as exc:
            rospy.logerr('Cannot send control command: %s' % exc)
            return (False or self.DEBUG)

        self.setTargetToCurrentState()
        for i, mk in enumerate(mask):
            if mk:
                self.targetState[i] = command.target[i]
        self.lastCommand['type'] = 1
        self.lastCommand['target'] = command.target
        self.lastCommand['mask'] = command.mask

        return True

    def move(self, speed=[0, 0, 0, 0, 0, 0]):
        """Tell robot to move with speed until stop sending command.

        Keyword Arguments:
            speed {list} -- [x, y, z, roll, pitch, yaw]
                x, y and z is in m/s.
                roll, pitch and yaw is in deg/s.
                (default: [0,0,0,0,0,0])

        Returns:
            {bool} -- Command sending status

        Note:
            Use right hand rules for angle input.
        """
        return (False or self.DEBUG)

    def stop(self):
        """Moving stop command

        Returns:
            {bool} -- Command sending status
        """
        return self.moveDist([0, 0, 0, 0, 0, 0], True)

    def getCurrent(self):
        """Get Current robot state

        Returns:
            list of float --  [x, y, z, roll, pitch, yaw]
        """
        if self.DEBUG:
            return [0.0 for _ in range(6)]
        resp = self.AUVStateProxy()
        AUVStateData = resp.auv_state.data  # Odometry
        orientation_q = AUVStateData.pose.pose.orientation
        orientation_list = [orientation_q.x,
                            orientation_q.y,
                            orientation_q.z,
                            orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        position_q = AUVStateData.pose.pose.position
        position_list = [position_q.x,
                         position_q.y,
                         position_q.z]
        return position_list+[roll, pitch, yaw]

    def setTargetToCurrentState(self):
        self.targetState = self.getCurrent()

    def rememberCurrent(self):
        """DEPRECATED
        It'll not be used after ROBOSUB2019
        and it'll throw an exception.
        """
        self.setTargetToCurrentState()

    def isOkay(self, acceptableDist=None, acceptableAngle=None):
        """Check is done last command

        Keyword Arguments:
            acceptableDist {float} -- acceptable dist in each axis in metre
            acceptableAngle {float} -- acceptable angle in each axis in degree

        Returns:
            {boolean} -- same method name if okay = True. Else False
        """
        if acceptableAngle is None:
            acceptableAngle = 7*math.pi/180
        else:
            acceptableAngle *= math.pi/180
        if acceptableDist is None:
            acceptableDist = 0.1
        if self.targetState is None:
            self.setTargetToCurrentState()
        curr = self.getCurrent()
        diff = [
            dat - curr[i]
            for i, dat in enumerate(self.targetState[0:3])
        ]
        diff2 = [
            self.angleDiff(curr[i+3], dat)
            for i, dat in enumerate(self.targetState[3:6])
        ]
        diff += diff2
        OKay = True
        lastCommand = self.lastCommand
        if lastCommand['type'] == 1:
            for i, cond in enumerate(lastCommand['mask']):
                if not cond:
                    continue
                if i < 3 and math.fabs(diff[i]) > acceptableDist:
                    OKay = False
                if i >= 3 and math.fabs(diff[i]) > acceptableAngle:
                    OKay = False
        return OKay

    def angleDiff(self, first, second):
        """Calculate difference between two angle. (second - first)

        Arguments:
            first {float} -- angle in first or old state. (rad)
            second {float} -- the new angle. (rad)
        """
        diff = second - first
        diff %= 2*math.pi
        if diff > math.pi:
            diff -= 2*math.pi
        if diff < -math.pi:
            diff += 2*math.pi

        return diff

    def calcNewPosition(self, current, command):
        """Calculate new position from user command
        It'll return new pos for sending to control.

        Arguments:
            current {list} -- current state
            command {list} -- command
        """
        x_plus = command[0]*math.cos(current[5]) - \
            command[1]*math.sin(current[5])
        y_plus = command[0]*math.sin(current[5]) + \
            command[1]*math.cos(current[5])

        newPos = [
            round(current[0]+x_plus, 6),
            round(current[1]+y_plus, 6),
            round(current[2]+command[2], 6),
            round((current[3]+command[3]) % (2*math.pi), 6),
            round((current[4]+command[4]) % (2*math.pi), 6),
            round((current[5]+command[5]) % (2*math.pi), 6)
        ]

        return newPos
