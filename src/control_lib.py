#! /usr/bin/python2


class Control:

    def moveDist(self, direction=[0, 0, 0, 0, 0, 0]):
        """Tell robot to move ... meter(s).

        Keyword Arguments:
            direction {list} -- [x, y, z, roll, pitch, yaw]
                (default: {[0,0,0,0,0,0]})

        Returns:
            {bool} -- Command sending status

        Note:
            Use right hand rules for angle input.
        """
        return False

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
        return False
