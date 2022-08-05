#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class RaspBotNode(Node):
    def __init__(self):
        super().__init__('raspbot_bringup')
        





def main(args=None):
    rclpy.init(args=args)
    r = RaspBotNode()
    rclpy.spin(r)
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
    
