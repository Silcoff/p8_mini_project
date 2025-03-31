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

        #initiate values
        self.laser_dist = 0
        self.laser_stp_ang = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.layser = self.create_subscription(LaserScan, '/scan', self.laser_to_2d,1)

        self.timer = self.create_timer(1.0, self.main_loop)

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
        from_frame_rel = 'base_scan'
        to_frame_rel = 'odom'
        try:
            transform = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1))
        except TransformException as ex:
            self.get_logger().info( f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
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

    def potential_field_gen(self,center_coord,field_strengt_constant):
        from_frame_rel = 'odom'
        to_frame_rel = 'base_link'
        try:
            transform = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1))
        except TransformException as ex:
            self.get_logger().info( f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        translation = transform.transform.translation
        robot_coord = [translation.x, translation.y]  


        dist = ((robot_coord[0]-center_coord[0])^2+(robot_coord[1]-center_coord[1])^2 )

        potential_field = field_strengt_constant * np.log(1/dist)  
        return potential_field


    def main_loop(self):
        self.laser_2_glob_coord()




def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
