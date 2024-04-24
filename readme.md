# ZLAC8015D ROS driver

## Installation and Usage

1. cd into your colcon workspace's soruce directory
2. `git clone --recursive https://github.com/Lexseal/zlac8015d_ros.git`
3. `cd zlac8015d_ros && cd ZLAC8015D_python && python3 -m pip install .`
4. go back to your colcon workspace and run `colcon build --symlink-install` and `source install/setup.bash`
5. `ros2 run zlac8015d_ros driver`
