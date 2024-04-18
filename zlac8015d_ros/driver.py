from time import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from zlac8015d import ZLAC8015D


class Driver(Node):
    def __init__(self):
        super().__init__("driver")

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        self.cmd_vel_sub  # prevent unused variable warning

        self.last_cmd_vel_time = time()
        self.drive_timer = self.create_timer(1 / 60, self.drive_callback)

        self.left_right_cmd_speed = np.zeros(2)
        self.robot_radius = 0.165

        self.motors = ZLAC8015D.Controller()

        self.motors.disable_motor()

        self.motors.set_accel_time(1000, 1000)
        self.motors.set_decel_time(1000, 1000)

        # if MODE == 1: print("Set relative position control")
        # elif MODE == 2: print("Set absolute position control")
        # elif MODE == 3: print("Set speed rpm control")
        self.motors.set_mode(3)
        self.motors.enable_motor()

    def cmd_vel_callback(self, msg):
        # Update the last received command velocity time
        self.last_cmd_vel_time = time()

        # Convert twist to left and right wheel speeds
        linear = msg.linear.x
        angular = msg.angular.z
        self.left_right_cmd_speed = linear + np.array(
            [-angular * self.robot_radius, angular * self.robot_radius]
        )
        # one wheel is backwards
        self.left_right_cmd_speed[0] = -self.left_right_cmd_speed[0]

    def drive_callback(self):
        # If we haven't received a command velocity message in the last second, stop the motors
        if time() - self.last_cmd_vel_time > 1:
            self.motors.set_speed(0, 0)
        else:
            self.motors.set_speed(*self.left_right_cmd_speed)


def main(args=None):
    rclpy.init(args=args)
    driver = Driver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
