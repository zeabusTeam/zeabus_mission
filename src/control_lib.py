#! /usr/bin/python2
import rospy
from zeabus_utility.srv import SendControlCommand
from zeabus_utility.msg import ControlCommand
from std_msgs.msg import Header


class Control:

    def __init__(self, senderName):
        """Control wrapper class for Mission Planner

        Keyword Arguments:
            senderName {str}
                -- Everyone that use this class must to privide your name.
        """
        rospy.init_node('MissionControlLib', anonymous=True)

        self.CtlCmdSrvName = 'SendControlCommand'
        self.seq = 0  # init first sequence no.
        self.senderName = rospy.get_name()+'.'+senderName

        rospy.loginfo('Waiting for %S srv to be available.' %
                      self.CtlCmdSrvName)
        rospy.wait_for_service(self.CtlCmdSrvName)
        self.proxy = rospy.ServiceProxy(
            self.CtlCmdSrvName, SendControlCommand)

    def moveDist(self, direction=[0, 0, 0, 0, 0, 0], ignoreZero=False):
        """Tell robot to move ... meter(s).

        Keyword Arguments:
            direction {list} -- [x, y, z, roll, pitch, yaw]
                (default: {[0,0,0,0,0,0]})

        Returns:
            {bool} -- Command sending status

        Note:
            Use right hand rules for angle input.
        """

        rospy.loginfo('Move %s command is executed.' % str(direction))

        head = Header()
        head.seq = self.seq
        head.stamp = rospy.Time.now()
        head.frame_id = self.senderName

        command = ControlCommand()
        command.header = head
        command.target = direction
        command.mask = [(ignoreZero or (x != 0)) for x in direction]

        self.seq += 1

        try:
            self.proxy(command)
        except rospy.ServiceException as exc:
            rospy.logerr('Cannot send control command: %s' % exc)
            return False

        return True

    def move(self, speed=[0, 0, 0, 0, 0, 0]):
        """Tell robot to move with speed until stop sending command.

        Keyword Arguments:
            speed {list} -- [x, y, z, roll, pitch, yaw]
                x, y and z is in m/s.
                roll, pitch and yaw is in deg/s.
                (default: {[0,0,0,0,0,0]})

        Returns:
            {bool} -- Command sending status

        Note:
            Use right hand rules for angle input.
        """
        return False

    def stop(self):
        """Moving stop command

        Returns:
            {bool} -- Command sending status
        """
        return self.moveDist([0, 0, 0, 0, 0, 0], True)
