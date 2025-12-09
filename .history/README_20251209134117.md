## Requirements

Install ROS Jazzy on ubuntu 24.04 and gazebo Harmonic, the original code was written br juliordzcer on https://github.com/juliordzcer/bebop_ros

### Install dependencies.
To be able to execute the programs it is necessary to install the following dependencies, executing the following commands in the console
```
sudo apt install ros-jazzy-tf-transformations
sudo apt install ros-jazzy-ament-lint-auto
sudo apt install ros-jazzy-ament-cmake
sudo apt install ros-jazzy-joy
```

### **Add Environment Variables and Source Setup File**

You can add the necessary environment variables and source file to your `.bashrc` file using the following commands:

```
echo "export GZ_SIM_RESOURCE_PATH=\$HOME/gz_bebop_ws/src/bebop_ros/bebop_gz/worlds:\$HOME/gz_bebop_ws/src/bebop_ros/bebop_gz/models" >> ~/.bashrc
echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=\$HOME/gz_bebop_ws/src/bebop_ros/bebop_gz/plugins/build:\${GZ_SIM_SYSTEM_PLUGIN_PATH}" >> ~/.bashrc
echo "export GZ_VERSION=harmonic" >> ~/.bashrc
source ~/.bashrc

```
### bebop_controller
Contains a position-based PID controller for single-agent trajectory tracking. Also includes an additional demo for swarm formation:
- Pyramid formation with leader-follower topology
- Straight-line movement for N agents

**Demo Commands:**
Single agent:
```
ros2 launch bebop_demo bebop1.launch.py
```

Swarm formation:

```
ros2 launch bebop_demo swarmbebop.launch.py
```

### bebop_demo
Contains utility packages and demonstration setups:
- `set_pose`: Sets initial drone positions
- `setpoint`: Generates circular trajectories for single drone operation
- Various executable examples
