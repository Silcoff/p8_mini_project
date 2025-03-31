import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class PotentialFieldController(Node):
    def __init__(self):
        super().__init__('potential_field_controller')

        # Subscribers
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.user_input_sub = self.create_subscription(Twist, '/cmd_user', self.user_input_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control Variables
        self.user_cmd = Twist()
        self.potential_field_cmd = Twist()
        self.control_weight = 0.5  # Adaptive weight

    def lidar_callback(self, msg):
        ang_inc = msg.angle_increment




    def user_input_callback(self, msg):
        self.user_cmd = msg

    def combined_control(self):
        human_control_canstant = 0.3 # has to be a value between 0 and 1. 0 human has no control
        final_control = human_control_canstant*self.human_control + (1-human_control_canstant)*self.robot_control
        self.cmd_pub.publish(final_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

