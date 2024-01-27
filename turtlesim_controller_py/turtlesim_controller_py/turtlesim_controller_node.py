#!/usr/bin/python3

import rclpy

from turtlesim_controller_py.turtlesim_controller import TurtlesimController

def main(args=None) -> None:
    rclpy.init(args=args)

    turtlesim_controller = TurtlesimController()

    rclpy.spin(turtlesim_controller)

    turtlesim_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
