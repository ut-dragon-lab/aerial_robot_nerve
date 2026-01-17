## Setup
### Installation
```bash
# Create ROS2 workspace
source /opt/ros/humble/setup.bash
mkdir uros_ws && cd uros_ws
# clone repository
vcs import src <<EOF
repositories:
  aerial_robot_nerve:
    type: git
    url: 'https://github.com/ut-dragon-lab/aerial_robot_nerve.git'
    version: master
EOF
# Install depended repositories for spinal
vcs import src < src/aerial_robot_nerve/spinal.repos
rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
```
### Generate micro ROS libraries
Run the following code. Please note that running this code requires Docker to be installed on your system.
```bash
python3 src/aerial_robot_nerve/spinal/scripts/make_microros_libraries.py --support-rtos
```
Then, all files required by micro ROS will be generated and placed in the appropriate directory within the STM32 project.
### Generate & Build micro ROS agent
```bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```