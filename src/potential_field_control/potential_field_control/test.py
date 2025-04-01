import math
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations

class FrameListener(Node):

    def __init__(self):
        super().__init__('test_for_tf')

        #initiate changing values
        self.laser_dist = 0
        self.laser_stp_ang = 0
        self.region_state = "safe"  # Track robot zone: safe, hysteresis, danger

        # Control Variables
        self.user_cmd = Twist()

        # initiate constants
        self.beta = 5.0  # Controls steepness of ks1
        self.gamma = 5.0  # Controls steepness of ks2
        self.epsilon = 0.01  # Small constant to avoid div by 0
        self.rd = 0.5    # Danger zone threshold
        self.rh = 0.7    # Hysteresis (safe zone buffer)



        # tf listener for transformation matrix
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscription
        self.layser = self.create_subscription(LaserScan, '/scan', self.laser_to_2d,1)
        self.user_input_sub = self.create_subscription(Twist, '/cmd_user', self.user_input_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # main loop on 1 sek timer
        self.timer = self.create_timer(1.0, self.main_loop)

    def user_input_callback(self, msg):
        self.user_cmd = msg

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
    def get_transform(self,from_frame, to_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1))
            return transform
        except TransformException as ex:
            self.get_logger().info( f'Could not transform {to_frame} to {from_frame}: {ex}')
            return

    def laser_to_2d(self,msg):
        coordinates = []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        for i, dist in enumerate(msg.ranges):
            angle = angle_min + i * angle_increment
            x = dist * math.cos(angle)
            y = dist * math.sin(angle)
            z = 0
            coordinates.append((x, y, z, 1))
        
        self.coordinates = np.array(coordinates)

    def laser_2_glob_coord(self):
        transform=self.get_transform('base_scan','odom')
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        # Convert quaternion to rotation matrix
        rotation_matrix = tf_transformations.quaternion_matrix([ rotation.x, rotation.y, rotation.z, rotation.w ])[:3, :3]  # Extract 3x3 rotation part
        # Create 4x4 transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix  # Set rotation
        transformation_matrix[:3, 3] = [translation.x, translation.y, translation.z]  


        global_coord = []
        for i,coord in enumerate(self.coordinates):
            global_coord.append(transformation_matrix.dot(coord))

        self.global_coord = np.array(global_coord)

    def potential_field_gen(self,robot_coord, center_coord,field_strengt_constant):
        dist = (math.pow((robot_coord[0]-center_coord[0]),2)+math.pow((robot_coord[1]-center_coord[1]),2) )

        if dist==0:
            return np.nan
        potential_field = field_strengt_constant * np.log(1/dist)  
        return potential_field

    def global_field(self,robot_coord,center_coord):

        accum_field = 1
        epsilon = 1e-6
        strength = 1
        tenth = 0
        for i, coord in enumerate(center_coord):
            tenth += i
            if tenth==10:
                field = self.potential_field_gen(robot_coord,coord,strength)
                if abs(field) < epsilon:
                    accum_field=accum_field
                else:
                    accum_field = accum_field * self.potential_field_gen(robot_coord,coord,strength)
                tenth = 0
            if i == 355:
                break
        return accum_field




    def desired_theta(self,robot_coord, center_coord):
        delta = 0.01
        # Calculate potential at slightly shifted points
        potential_x_plus = self.global_field((robot_coord[0] + delta, robot_coord[1]),center_coord)
        potential_x_minus = self.global_field((robot_coord[0] - delta, robot_coord[1]),center_coord)
        potential_y_plus = self.global_field((robot_coord[0], robot_coord[1] + delta),center_coord)
        potential_y_minus = self.global_field((robot_coord[0], robot_coord[1] - delta),center_coord)

        # Calculate numerical partial derivatives (gradient components)
        partial_phi_dx = (potential_x_plus - potential_x_minus) / (2 * delta)
        partial_phi_dy = (potential_y_plus - potential_y_minus) / (2 * delta)

        desired_theta = math.atan2(-partial_phi_dy, -partial_phi_dx)

        return desired_theta

    def desired_theta_dot(self,robot_coord,center_coord,robot_theta,robot_linear_velocity_v):

        delta = 0.01
        robot_x = robot_coord[0]
        robot_y = robot_coord[1]
         # --- 1. Calculate Robot Velocity Components [vx, vy] ---
        # vx = v * cos(theta)
        # vy = v * sin(theta)
        vx = robot_linear_velocity_v * math.cos(robot_theta)
        vy = robot_linear_velocity_v * math.sin(robot_theta)

        # --- 2. Numerically Estimate Spatial Gradient of target_angle ---
        #    [ d(target_angle)/dx ,  d(target_angle)/dy ]

        # Calculate target angle at points slightly shifted in x and y
        # Pass any extra arguments needed by the target angle function
        theta_d_xp = self.desired_theta((robot_x + delta, robot_y),center_coord)
        theta_d_xm = self.desired_theta((robot_x - delta, robot_y),center_coord)
        theta_d_yp = self.desired_theta((robot_x, robot_y + delta),center_coord)
        theta_d_ym = self.desired_theta((robot_x, robot_y - delta),center_coord)

        # Calculate the *shortest* difference between angles to handle wrapping (-pi to pi)
        # diff = atan2(sin(angle1 - angle2), cos(angle1 - angle2))
        diff_theta_x = math.atan2(math.sin(theta_d_xp - theta_d_xm), math.cos(theta_d_xp - theta_d_xm))
        diff_theta_y = math.atan2(math.sin(theta_d_yp - theta_d_ym), math.cos(theta_d_yp - theta_d_ym))

        # Estimate partial derivatives using the central difference formula
        # d(target_angle)/dx ≈ diff_theta_x / (2 * delta)
        # d(target_angle)/dy ≈ diff_theta_y / (2 * delta)
        partial_target_angle_dx = diff_theta_x / (2.0 * delta)
        partial_target_angle_dy = diff_theta_y / (2.0 * delta)

        # The estimated spatial gradient vector is [partial_target_angle_dx, partial_target_angle_dy]

        # --- 3. Compute the Dot Product: gradient ⋅ velocity ---
        # theta_d_dot = (d(target_angle)/dx * vx) + (d(target_angle)/dy * vy)
        theta_d_dot = (partial_target_angle_dx * vx) + (partial_target_angle_dy * vy)

        return theta_d_dot


    def main_loop(self):
        self.laser_2_glob_coord()

        transform = self.get_transform('odom','base_link')
        translation = transform.transform.translation
        robot_coord = [translation.x, translation.y]  
        rotation = transform.transform.rotation
        # Convert quaternion to rotation matrix
        rotation_matrix = tf_transformations.quaternion_matrix([ rotation.x, rotation.y, rotation.z, rotation.w ])[:3, :3]  # Extract 3x3 rotation part
        robot_theta = rotation_matrix[2,0]


        vel=0.2

        print(self.desired_theta_dot(robot_coord,self.global_coord, robot_theta,vel))



def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
