# Parrot Bebop 2 Gazebo Simulation

## Requirements

Install [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) on Ubuntu 22.04.

The original code was written by juliordzcer [here](https://github.com/juliordzcer/bebop_ros)

## Necessary Dependencies

### Gazebo Harmonic
Run the following commands to install Gazebo Harmonic
```
sudo apt update && sudo apt upgrade
sudo apt-get install curl lsb-release gnupg
```

Then, install Gazebo
```
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
sudo apt install libgps-dev
sudo apt install libyaml-cpp-dev
```

### Install ROS Packages
```
sudo apt install ros-humble-tf-transformations
sudo apt install ros-humble-ament-lint-auto
sudo apt install ros-humble-ament-cmake
sudo apt install ros-humble-joy
```

```
pip3 install transforms3d
```

### Clone Repo, Initialize Submodules
```
cd /<your_workspace>/src
git clone https://github.com/UFL-Autonomy-Park/bebop_gz.git
git submodule update --init --recursive
```

### Build Bebop Gazebo Plugin
```
cd bebop_gz/plugins
mkdir build
cmake ..
cd /<your_workspace>
```

## Environment Variables
Run the following
```
echo "export GZ_SIM_RESOURCE_PATH=\$HOME/<your_workspace>/src/bebop_gz/worlds:\$HOME/bebop_sim/src/bebop_gz/models" >> ~/.bashrc
echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=\$HOME/<your_workspace>/src/bebop_gz/plugins/build:\${GZ_SIM_SYSTEM_PLUGIN_PATH}" >> ~/.bashrc
echo "export GZ_VERSION=harmonic" >> ~/.bashrc
source ~/.bashrc
```

## Build
```
colcon build --packages-skip vision_msgs_rviz_plugins
source install/setup.bash
```

**Demo commands:**
Single agent simulation:
```
ros2 launch bebop_demo bebop1.launch.py
```
