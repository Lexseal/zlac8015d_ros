# ZLAC8015D ROS driver

## Installation and Usage

1. cd into your colcon workspace's soruce directory
2. `git clone --recursive https://github.com/Lexseal/zlac8015d_ros.git`
3. `cd zlac8015d_ros && cd ZLAC8015D_python && python3 -m pip install .`
4. go back to your colcon workspace and run `colcon build --symlink-install` and `source install/setup.bash`
5. `ros2 run zlac8015d_ros driver <optional port like /dev/serial/by-id/...>`

## Note

This uses RS485-USB converter to communicate with the driver. The default port is `/dev/ttyUSB0`, you can change it in `driver.py` file. It is likely you will need to run `sudo usermod -a -G dialout $USER` or `sudo chmod 777 /dev/ttyUSB0` to have permission to access the port.

## Configuration

You will likely need to tune the robot radius inside the `driver.py`. Default is 0.165 (meters). If you are using `get_linear_velocities()`, you can pass in an argument called `wheel_radius` when initializing `self.motors`. The default value is 0.065 (meters).
