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
 

        self.roj = 0.37  # Radius of obstacle
        self.rd = 0.5    # Danger zone threshold
        self.rh = 0.7    # Hysteresis (safe zone buffer)

        self.beta = 5.0  # Controls steepness of ks1
        self.gamma = 5.0  # Controls steepness of ks2
        self.epsilon = 0.01  # Small constant to avoid div by 0

        self.region_state = "safe"  # Track robot zone: safe, hysteresis, danger
        self.ks = 0.0  # Control blending factor


    def lidar_callback(self, msg):
        # Convert LIDAR scan to potential field avoidance command
        min_distance = min(msg.ranges)
        angle_index = np.argmin(msg.ranges)

        self.update_region_state(min_distance)
        self.compute_ks(min_distance)


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

    def update_region_state(self, d):
        prev = self.region_state

        if self.region_state == "safe":
            if d <= self.rd:
                self.region_state = "danger"
            elif d <= self.rh:
                self.region_state = "hysteresis"
        elif self.region_state == "hysteresis":
            if d <= self.rd:
                self.region_state = "danger"
            elif d > self.rh:
                self.region_state = "safe"
        elif self.region_state == "danger":
            if d > self.rh:
                self.region_state = "safe"
            elif d > self.rd:
                self.region_state = "hysteresis"

        # If region changed, store previous
        if prev != self.region_state:
            self.previous_region_state = prev
            self.get_logger().info(f"Region changed: {prev} → {self.region_state}")

    def compute_ks(self, d):
        if self.region_state == "danger":
            x = (self.rd - d) / (self.rd - self.roj + self.epsilon)
            self.ks = np.tanh(self.beta * x)
        elif self.region_state == "hysteresis":
            if self.previous_region_state == "danger":
                # Entered from Md → use ks2
                x = (self.rh - d) / (self.rh - self.rd + self.epsilon)
                self.ks = np.tanh(self.epsilon) * np.tanh(self.gamma * x)
            else:
                # Entered from Ms → reset to 0
                self.ks = 0.0
        else:
            self.ks = 0.0

    def compute_final_command(self):
        # ks is already computed
        final_cmd = Twist()
        final_cmd.linear.x = (1 - self.ks) * self.user_cmd.linear.x + self.ks * self.potential_field_cmd.linear.x
        final_cmd.angular.z = (1 - self.ks) * self.user_cmd.angular.z + self.ks * self.potential_field_cmd.angular.z

        self.cmd_pub.publish(final_cmd)
'''
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
'''
def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

