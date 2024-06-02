from time import sleep
import sys

import numpy as np
import rclpy
import transforms3d as t3d
from geometry_msgs.msg import (
    PoseWithCovariance,
    TransformStamped,
    Twist,
    TwistWithCovariance,
)
from nav_msgs.msg import Odometry, Imu
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
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
        self.imu_pub = self.create_publisher(Imu, "/imu", 10)

        self.last_cmd_vel_time = self.get_clock().now()
        self.drive_timer = self.create_timer(1 / 60, self.drive_callback)

        self.left_right_cmd_speed = np.zeros(2)
        self.robot_radius = 0.165

        port = "/dev/ttyUSB0" if len(sys.argv) < 2 else sys.argv[1]
        self.motors = ZLAC8015D.Controller(port=port)

        self.motors.disable_motor()

        self.motors.set_accel_time(100, 100)
        self.motors.set_decel_time(50, 50)

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

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu_link"
        self.imu_msg.orientation_covariance = np.eye(3).flatten().tolist()
        self.imu_msg.angular_velocity_covariance = np.eye(3).flatten().tolist()
        self.imu_msg.linear_acceleration_covariance = np.eye(3).flatten().tolist()

        self.odom_calc_timer = self.create_timer(1 / 100, self.odom_calc_callback)
        self.tf_pub_timer = self.create_timer(1 / 50, self.tf_pub_callback)

        self.transform_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_odom = TransformStamped()
        self.tf_odom.header.frame_id = "odom"
        self.tf_odom.child_frame_id = "base_link"
        self.tf_footprint = TransformStamped()
        self.tf_footprint.header.frame_id = "base_link"
        self.tf_footprint.child_frame_id = "base_footprint"
        # constant transform of -3cm in z
        self.tf_footprint.transform.translation.z = -0.03

        self.cur_x, self.cur_y, self.cur_z = 0, 0, 0
        self.last_speed = 0

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
        dt = (cur_time - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = cur_time
        
        left_speed, right_speed = self.motors.get_linear_velocities()
        left_speed *= self.motor_polarity[0]
        right_speed *= self.motor_polarity[1]
        
        R = self.robot_radius * (left_speed + right_speed) / (right_speed - left_speed)
        omega = (right_speed - left_speed) / (2*self.robot_radius)
        ICC = self.cur_x - R*np.sin(self.cur_z), self.cur_y + R*np.cos(self.cur_z)
        Mat = np.array([[np.cos(omega*dt), -np.sin(omega*dt), 0],
                        [np.sin(omega*dt), np.cos(omega*dt), 0],
                        [0, 0, 1]])
        new_pose = Mat @ np.array([self.cur_x - ICC[0], self.cur_y - ICC[1], self.cur_z]) + np.array([ICC[0], ICC[1], omega*dt])
        self.cur_x, self.cur_y, self.cur_z = new_pose
        
        self.imu_msg.header.stamp = cur_time.to_msg()
        # angular velocity only in z direction
        self.imu_msg.angular_velocity.z = omega
        cur_speed = (right_speed + left_speed) / 2
        self.imu_msg.linear_acceleration.x = (cur_speed - self.last_speed) / dt
        self.last_speed = cur_speed
        self.imu_pub.publish(self.imu_msg)

    def tf_pub_callback(self):
        self.pose_with_covariance.pose.position.x = self.cur_x
        self.pose_with_covariance.pose.position.y = self.cur_y
        q = t3d.euler.euler2quat(0, 0, self.cur_z)
        self.pose_with_covariance.pose.orientation.w = q[0]
        self.pose_with_covariance.pose.orientation.x = q[1]
        self.pose_with_covariance.pose.orientation.y = q[2]
        self.pose_with_covariance.pose.orientation.z = q[3]

        self.tf_odom.header.stamp = self.odom_msg.header.stamp
        self.tf_odom.transform.translation.x = self.pose_with_covariance.pose.position.x
        self.tf_odom.transform.translation.y = self.pose_with_covariance.pose.position.y
        self.tf_odom.transform.rotation = self.pose_with_covariance.pose.orientation

        self.odom_pub.publish(self.odom_msg)
        self.transform_broadcaster.sendTransform(self.tf_odom)
        self.static_broadcaster.sendTransform(self.tf_footprint)

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
