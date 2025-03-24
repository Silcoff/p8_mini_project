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
        # Convert LIDAR scan to potential field avoidance command
        min_distance = min(msg.ranges)
        angle_index = np.argmin(msg.ranges)

        if min_distance < 0.5:  # If obstacle is within 50 cm
            self.potential_field_cmd.linear.x = 0.0
            self.potential_field_cmd.angular.z = np.sign(angle_index - len(msg.ranges)/2) * 0.5  # Turn away
        else:
            self.potential_field_cmd.linear.x = 0.2
            self.potential_field_cmd.angular.z = 0.0

        self.compute_final_command()

    def user_input_callback(self, msg):
        self.user_cmd = msg
        self.compute_final_command()

    def compute_final_command(self):
        # Adjust control weight based on proximity to obstacles
        if abs(self.potential_field_cmd.angular.z) > 0:
            self.control_weight = 1.0  # Full autonomy near obstacles
        else:
            self.control_weight = 0.2  # More human control away from obstacles

        # Shared control blend
        final_cmd = Twist()
        final_cmd.linear.x = (1 - self.control_weight) * self.user_cmd.linear.x + self.control_weight * self.potential_field_cmd.linear.x
        final_cmd.angular.z = (1 - self.control_weight) * self.user_cmd.angular.z + self.control_weight * self.potential_field_cmd.angular.z

        self.cmd_pub.publish(final_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

