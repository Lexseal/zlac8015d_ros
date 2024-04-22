from time import sleep

import numpy as np
import rclpy
import transforms3d as t3d
from geometry_msgs.msg import (
    PoseWithCovariance,
    TransformStamped,
    Twist,
    TwistWithCovariance,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from zlac8015d import ZLAC8015D


class Driver(Node):
    def __init__(self):
        super().__init__("driver")

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        self.motor_polarity = np.array([-1, 1])

        self.cmd_vel_sub  # prevent unused variable warning

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        self.last_cmd_vel_time = self.get_clock().now()
        self.drive_timer = self.create_timer(1 / 60, self.drive_callback)

        self.left_right_cmd_speed = np.zeros(2)
        self.robot_radius = 0.165

        self.motors = ZLAC8015D.Controller()

        self.motors.disable_motor()

        self.motors.set_accel_time(1000, 1000)
        self.motors.set_decel_time(500, 500)

        # if MODE == 1: print("Set relative position control")
        # elif MODE == 2: print("Set absolute position control")
        # elif MODE == 3: print("Set speed rpm control")
        self.motors.set_mode(3)
        self.motors.enable_motor()

        self.last_odom_time = self.get_clock().now()
        self.twist_with_covariance = TwistWithCovariance()
        self.twist_with_covariance.covariance = np.eye(6).flatten().tolist()
        self.pose_with_covariance = PoseWithCovariance()
        # well this is a bad guess since the cov will get larger as time goes
        # but no one should use the odom pose anyway since it's not accurate
        self.pose_with_covariance.covariance = np.eye(6).flatten().tolist()
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"
        self.odom_msg.twist = self.twist_with_covariance
        self.odom_msg.pose = self.pose_with_covariance

        self.odom_calc_timer = self.create_timer(1 / 60, self.odom_calc_callback)
        self.tf_pub_timer = self.create_timer(1 / 10, self.tf_pub_callback)

        self.transform_broadcaster = TransformBroadcaster(self)
        self.tf_odom = TransformStamped()
        self.tf_odom.header.frame_id = "odom"
        self.tf_odom.child_frame_id = "base_link"
        self.tf_footprint = TransformStamped()
        self.tf_footprint.header.frame_id = "base_link"
        self.tf_footprint.child_frame_id = "footprint"
        # constant transform of -3cm in z
        self.tf_footprint.transform.translation.z = -0.03

    def cmd_vel_callback(self, msg):
        # Update the last received command velocity time
        self.last_cmd_vel_time = self.get_clock().now()

        # Convert twist to left and right wheel speeds
        linear = msg.linear.x
        angular = msg.angular.z
        self.left_right_cmd_speed = linear + np.array(
            [-angular * self.robot_radius, angular * self.robot_radius]
        )
        self.left_right_cmd_speed *= self.motor_polarity

    def drive_callback(self):
        # If we haven't received a command velocity message in the last second, stop the motors
        cur_time = self.get_clock().now()
        if (cur_time - self.last_cmd_vel_time).nanoseconds / 1e9 > 1:
            self.motors.set_speed(0, 0)
        else:
            self.motors.set_speed(*self.left_right_cmd_speed)

    def odom_calc_callback(self):
        cur_time = self.get_clock().now()
        self.odom_msg.header.stamp = cur_time.to_msg()
        left_speed, right_speed = self.motors.get_linear_velocities()
        left_speed *= self.motor_polarity[0]
        right_speed *= self.motor_polarity[1]
        avg_speed = (left_speed + right_speed) / 2
        self.twist_with_covariance.twist.linear.x = avg_speed
        omega = (right_speed - left_speed) / (2 * self.robot_radius)
        self.twist_with_covariance.twist.angular.z = omega

        dt = (cur_time - self.last_odom_time).nanoseconds / 1e9
        cur_heading = self.pose_with_covariance.pose.orientation.z
        dx = avg_speed * np.cos(cur_heading) * dt
        dy = avg_speed * np.sin(cur_heading) * dt
        self.pose_with_covariance.pose.position.x += dx
        self.pose_with_covariance.pose.position.y += dy
        self.pose_with_covariance.pose.orientation.z += omega * dt

        self.last_odom_time = cur_time

    def tf_pub_callback(self):
        self.odom_pub.publish(self.odom_msg)

        self.tf_odom.header.stamp = self.odom_msg.header.stamp
        self.tf_odom.transform.translation.x = self.pose_with_covariance.pose.position.x
        self.tf_odom.transform.translation.y = self.pose_with_covariance.pose.position.y
        quat = t3d.euler.euler2quat(
            self.pose_with_covariance.pose.orientation.x,
            self.pose_with_covariance.pose.orientation.y,
            self.pose_with_covariance.pose.orientation.z,
        )
        self.tf_odom.transform.rotation.w = quat[0]
        self.tf_odom.transform.rotation.x = quat[1]
        self.tf_odom.transform.rotation.y = quat[2]
        self.tf_odom.transform.rotation.z = quat[3]
        self.transform_broadcaster.sendTransform([self.tf_odom, self.tf_footprint])

    def destroy_node(self):
        self.motors.disable_motor()
        sleep(1)  # wait for the motors to stop
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    driver = Driver()
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
