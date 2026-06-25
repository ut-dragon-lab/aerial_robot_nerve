# Setup

## Create ROS2 workspace
! NOTE: When building this package while building `aerial_robot_base`, skip creating a new workspace and clone the repo in the existing workspace. 
```bash
source /opt/ros/humble/setup.bash
mkdir -p uros_ws/src && cd uros_ws
```
## Clone repository
```bash
# Add repository to the workspace
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
## Generate micro ROS libraries
! If you are using a Docker container to run ROS 2, please execute the following outside the container on your local machine, as it itself relies on pulling a Docker image.

The following code requires Docker to be installed on your system.
```bash
python3 src/aerial_robot_nerve/spinal/scripts/make_microros_libraries.py --support_rtos
```
Then, all files required by micro ROS will be generated and placed in the appropriate directory within the STM32 project.
## Generate & Build micro ROS agent
```bash
source install/setup.bash  # setup.zsh if using zsh
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

