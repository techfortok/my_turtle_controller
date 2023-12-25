#!/usr/bin/python3

import rospy

from turtlesim_controller import TurtlesimController


def main() -> None:

    turtlesim_controller = TurtlesimController()

    try:
        turtlesim_controller.process()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
