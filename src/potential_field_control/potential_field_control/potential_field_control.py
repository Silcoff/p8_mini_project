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


        self.beta = 5.0  # Controls steepness of ks1
        self.gamma = 5.0  # Controls steepness of ks2
        self.epsilon = 0.01  # Small constant to avoid div by 0
        self.rd = 0.5    # Danger zone threshold
        self.rh = 0.7    # Hysteresis (safe zone buffer)

        self.region_state = "safe"  # Track robot zone: safe, hysteresis, danger



    def user_input_callback(self, msg):
        self.user_cmd = msg
        self.compute_final_command()

    def update_region_state(self, dist):
        prev = self.region_state

        if self.region_state == "safe":
            if dist <= self.rd:
                self.region_state = "danger"
            elif dist <= self.rh:
                self.region_state = "hysteresis"
        elif self.region_state == "hysteresis":
            if dist <= self.rd:
                self.region_state = "danger"
            elif dist > self.rh:
                self.region_state = "safe"
        elif self.region_state == "danger":
            if dist > self.rh:
                self.region_state = "safe"
            elif dist > self.rd:
                self.region_state = "hysteresis"

        # If region changed, store previous
        if prev != self.region_state:
            self.previous_region_state = prev
            self.get_logger().info(f"Region changed: {self.region_state} → {self.region_state}")

    def compute_ks(self, dist):
        self.update_region_state(dist)
        
        if self.region_state == "danger":
            ks = np.tanh(self.beta * ((self.rd - dist) / self.rd) + self.epsilon)
        elif self.region_state == "hysteresis":
            if self.previous_region_state == "danger":
                # Entered from Md → use ks2
                ks = np.tanh(self.epsilon) * np.tanh(self.gamma * ((self.rh - dist) / (self.rh - self.rd)))
            else:
                # Entered from Ms → reset to 0
                ks = 0.0
        else:
            ks = 0.0

        return ks

    def compute_final_command(self):
        final_cmd = Twist()

        lidar_min_val = 0
        self.compute_ks(lidar_min_val)

        self.cmd_pub.publish(final_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

