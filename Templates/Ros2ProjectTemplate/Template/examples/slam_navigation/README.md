# ROS2 Warehouse demo

This project demonstrates the usage of ROS2 along with the O3DE simulation.

## Requirements

Refer to the [O3DE System Requirements](https://www.o3de.org/docs/welcome-guide/requirements/) documentation to make sure that the system/hardware requirements are met. 
This project has the following dependencies:

- [O3DE](https://github.com/o3de/o3de) 
- [ROS2 Gem](https://github.com/RobotecAI/o3de-ros2-gem)

Please make sure that `clang` was installed and configured. For details refer to [this section](https://www.o3de.org/docs/welcome-guide/requirements/#linux) of O3DE documentation.

To run the navigation example, two ROS2 packages are also required:
- [navigation2](https://github.com/ros-planning/navigation2)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)

## Setup Instructions

The following steps will assume the following:

- All of the requirements coming from O3DE are met.
- You have ROS2 [installed](https://docs.ros.org/en/humble/Installation.html) and environment is [sourced](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files).
- The instructions will be based on a common base folder: `$DEMO_BASE`. For convenience you can export chosen directory name to `$DEMO_BASE`, for example:
```shell
export DEMO_BASE=/home/${USER}/github
mkdir -p ${DEMO_BASE}
```

### 1. Install ROS2 packages

```shell
sudo apt install ros-${ROS_DISTRO}-slam-toolbox ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-pointcloud-to-laserscan ros-${ROS_DISTRO}-teleop-twist-keyboard ros-${ROS_DISTRO}-ackermann-msgs ros-${ROS_DISTRO}-gazebo-msgs ros-${ROS_DISTRO}-control-toolbox
```


### 2. Clone O3DE and register the engine

```shell
cd $DEMO_BASE
git clone https://github.com/o3de/o3de.git
cd $DEMO_BASE/o3de
git lfs install
git lfs pull
$DEMO_BASE/o3de/scripts/o3de.sh register --this-engine
```

### 3. Clone and register the ROS2 Gem locally
ROS2 Gem is an interface between O3DE and ROS2.

```shell
cd $DEMO_BASE
git clone https://github.com/RobotecAI/o3de-ros2-gem.git
$DEMO_BASE/o3de/scripts/o3de.sh register --gem-path $DEMO_BASE/o3de-ros2-gem
```

### 4. Clone and register this project and build it

```shell
cd $DEMO_BASE
git clone https://github.com/RobotecAI/Ros2WarehouseDemo.git
$DEMO_BASE/o3de/scripts/o3de.sh register -pp $DEMO_BASE/Ros2WarehouseDemo/Project
```

Register gems included in this project.

```shell
$DEMO_BASE/o3de/scripts/o3de.sh register --gem-path $DEMO_BASE/Ros2WarehouseDemo/Gems/WarehouseSample
$DEMO_BASE/o3de/scripts/o3de.sh enable-gem -gn WarehouseSample -pp $DEMO_BASE/Ros2WarehouseDemo/Project

$DEMO_BASE/o3de/scripts/o3de.sh register --gem-path $DEMO_BASE/Ros2WarehouseDemo/Gems/RosRobotSample
$DEMO_BASE/o3de/scripts/o3de.sh enable-gem -gn RosRobotSample -pp $DEMO_BASE/Ros2WarehouseDemo/Project
```

Next, let us the build project with the necessary elements of the O3DE engine and ROS2 Gem.
```shell
cd $DEMO_BASE/Ros2WarehouseDemo/Project
cmake -B build/linux -G "Ninja Multi-Config" -DLY_UNITY_BUILD=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DLY_PARALLEL_LINK_JOBS=16 -DLY_STRIP_DEBUG_SYMBOLS=OFF
cmake --build build/linux --config profile --target Ros2WarehouseDemo.GameLauncher Editor
```

### 5. Launch Editor
Finally, previously built O3DE with preloaded ROS2 Gem can be run:
```shell
$DEMO_BASE/Ros2WarehouseDemo/Project/build/linux/bin/profile/Editor
```

## Robot teleoperation

In this example, the simulated robot is controlled with the simple keyboard teleoperation tool provided by ROS2.
Before you continue, please make sure that you have the ROS2 environment correctly [installed](https://docs.ros.org/en/humble/Installation.html) and [sourced](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files).

### 1. Launch Editor

```shell
$DEMO_BASE/Ros2WarehouseDemo/Project/build/linux/bin/profile/Editor
```

### 2. Run simulation

1. In `O3DE` editor open `DemoLevel`
1. Start the simulation by clicking the `Play Game` button or pressing `CTRL+G`


### 3. Start teleoperation node

```shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
With keys `i`, `j`, `k`, `m`, `,`,`.` you should be able to drive around the scene with the simulated robot.

Alternatively, node from package `rqt_robot_steering` can be used:
```shell
ros2 run rqt_robot_steering rqt_robot_steering 
```
## Running ROS2 navigation example

In this example, the simulation and the ROS2 navigation stack are running together. While running, a map is being built, and the robot is controllable via target goal requests.

### 1. Launch Editor

```shell
$DEMO_BASE/Ros2WarehouseDemo/Project/build/linux/bin/profile/Editor
```
### 2. Run the simulation

1. In `O3DE` editor open `DemoLevel`
1. Start the simulation by clicking the `Play Game` button or pressing `CTRL+G`

### 3. Run the ROS2 navigation stack

```shell
cd $DEMO_BASE/Ros2WarehouseDemo/launch
ros2 launch navigation.launch.py
```

### 4. Set robot target goal

Use RViz2 GUI to set the goal by using the `2D Goal Pose` tool (upper toolbar). 

## Troubleshooting

If launching Ros2WarehouseDemo Editor takes a lot of time and does not work properly make sure that you have cloned Ros2WarehouseDemo repository after installing git lfs. Otherwise run `git lfs pull` manually inside the directory.
