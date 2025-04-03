#!/usr/bin/env python

import os
import select
import sys

from geometry_msgs.msg import Twist
import rclpy
from rclpy.qos import QoSProfile
import threading
import pygame


if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, x : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""
pressed_keys = set()
running = True

'''ef on_press(key):
    try:
        pressed_keys.add(key.char.lower())
    except AttributeError:
        pass  # Special keys

def on_release(key):
    global running
    try:
        pressed_keys.discard(key.char.lower())
    except AttributeError:
        pass
    if key == keyboard.Key.esc:
        running = False
        return False  # Stops listener
sta
def on_press(key):
    try:
        pressed_keys.add(key.char.lower())
    except AttributeError:
        pass  # Special keys

def on_release(key):
    global running
    try:
        pressed_keys.discard(key.char.lower())
    except AttributeError:
        pass
    if key == keyboard.Key.esc:
        running = False
        return False  # Stops listener
'''

# Start the listener in a background thread
#listener = keyboard.Listener(on_press=on_press, on_release=on_release)
#listener.start()


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity,
        target_angular_velocity))


def make_simple_profile(output_vel, input_vel, slop):
    if input_vel > output_vel:
        output_vel = min(input_vel, output_vel + slop)
    elif input_vel < output_vel:
        output_vel = max(input_vel, output_vel - slop)
    else:
        output_vel = input_vel

    return output_vel


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'waffle':
        return constrain(velocity, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
        return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)



def check_angular_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'waffle':
        return constrain(velocity, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
        return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)


def main():
    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, '/manual_input', qos)

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    running = True
    print(msg)
    try:
        while running and rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            keys = pygame.key.get_pressed()
            target_linear_velocity = 0.0
            target_angular_velocity = 0.0

            if keys[pygame.K_w]:
                target_linear_velocity = check_linear_limit_velocity(0.1)
            if keys[pygame.K_s]:
                target_linear_velocity = check_linear_limit_velocity(-0.1)
            if keys[pygame.K_a]:
                target_angular_velocity = check_angular_limit_velocity(0.3)
            if keys[pygame.K_d]:
                target_angular_velocity = check_angular_limit_velocity(-0.3)
            if keys[pygame.K_SPACE] or keys[pygame.K_x]:
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()

            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            twist.linear.x = control_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_velocity

            pub.publish(twist)

            

            print_vels(control_linear_velocity, control_angular_velocity)

            rclpy.spin_once(node, timeout_sec=0.1)

    except Exception as e:
        print("Error:", e)

    finally:
        twist = Twist()
        pub.publish(twist)
        node.destroy_node()
        pygame.quit()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
