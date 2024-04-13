import rospy
from geometry_msgs.msg import Twist

from zlac8015d import ZLAC8015D

import numpy as np

class ZLAC8015D:
    def __init__(self):
        rospy.init_node('ZLAC8015D', anonymous=True)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.last_cmd_vel_time = rospy.get_time()
        self.drive_timer = rospy.Timer(rospy.Duration(1.0/60.0), self.drive_callback)  # 60Hz timer

        self.left_right_rpm = np.zeros(2)
        self.radius = 0.126
        self.wheel_diamaeter = 0.1
        self.wheel_cirumference = np.pi * self.wheel_diamaeter

        motors = ZLAC8015D.Controller(port="/dev/ttyUSB0")

        motors.disable_motor()

        motors.set_accel_time(1000,1000)
        motors.set_decel_time(1000,1000)

        # if MODE == 1:
		# 	print("Set relative position control")
		# elif MODE == 2:
		# 	print("Set absolute position control")
		# elif MODE == 3:
		# 	print("Set speed rpm control")
        motors.set_mode(3)
        motors.enable_motor()

    def cmd_vel_callback(self, msg):
        # Update the last received command velocity time
        self.last_cmd_vel_time = rospy.get_time()

        # Convert twist to left and right wheel speeds
        linear = msg.linear.x
        angular = msg.angular.z
        left_right_speed = linear + np.array([-angular * self.radius, angular * self.radius])
        self.left_right_rpm = left_right_speed / self.wheel_cirumference * 60

    def drive_callback(self):
        # If we haven't received a command velocity message in the last second, stop the motors
        if rospy.get_time() - self.last_cmd_vel_time > 1.0:
            self.motors.set_rpm(0,0)
        else:
            self.motors.set_rpm(*self.left_right_rpm)

if __name__ == '__main__':
    node = ZLAC8015D()
    rospy.spin()
