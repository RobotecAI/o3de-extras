# ROS2 Project Template

This template allows to create a ROS2 project with sample content. 

The example ROS2 navigation stack launchfile is bundled with the template.

## Requirements

Refer to the [O3DE System Requirements](https://www.o3de.org/docs/welcome-guide/requirements/) documentation to make sure that the system/hardware requirements are met. 
This project has the following dependencies:

- [O3DE](https://github.com/o3de/o3de)
- [ROS2](https://www.ros.org/) (galactic or humble)
- [ROS2 Gem](https://github.com/o3de/o3de-extras/tree/development/Gems/ROS2)
- [RosRobotSample Assets](https://github.com/o3de/o3de-extras/tree/development/Gems/RosRobotSample)
- [WarehouseSample Assets](https://github.com/o3de/o3de-extras/tree/development/Gems/WarehouseSample)

Please make sure that `clang` was installed and configured. For details refer to [this section](https://www.o3de.org/docs/welcome-guide/requirements/#linux) of O3DE documentation.

To run the navigation example, two ROS2 packages are also required:
- [navigation2](https://github.com/ros-planning/navigation2)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)

## Setup Instructions

The following steps will assume the following:

- All of the requirements coming from O3DE are met.
- You have ROS2 humble [installed](https://docs.ros.org/en/humble/Installation.html) and environment is [sourced](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files).
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

### 3. Clone the `o3de-extras` repository containing the template and asset gems

```shell
cd $DEMO_BASE
git clone git@github.com:o3de/o3de-extras.git
```

Register gems included in this project.

```shell
$DEMO_BASE/o3de/scripts/o3de.sh register --gem-path $DEMO_BASE/o3de-extras/Gems/ROS2
$DEMO_BASE/o3de/scripts/o3de.sh register --gem-path $DEMO_BASE/o3de-extras/Gems/WarehouseSample
$DEMO_BASE/o3de/scripts/o3de.sh register --gem-path $DEMO_BASE/o3de-extras/Gems/RosRobotSample
```

### 4. Create a ROS2 project from the template

Assign a name for the new project. In this example, it is assumed that it will be: `WarehouseTest`, and it will be located in `$DEMO_BASE/WarehouseTest` folder. 

```shell
export PROJECT_NAME=WarehouseTest
export PROJECT_PATH=$DEMO_BASE/$PROJECT_NAME
$DEMO_BASE/o3de/scripts/o3de.sh create-project --project-path $PROJECT_PATH --template-path $DEMO_BASE/o3de-extras/Templates/Ros2ProjectTemplate/ -f --keep-restricted-in-project --no-register
$DEMO_BASE/o3de/scripts/o3de.sh register --project-path $PROJECT_PATH/Project
```

Enable gems.

```shell
$DEMO_BASE/o3de/scripts/o3de.sh enable-gem --gem-name ROS2 --project-path $PROJECT_PATH/Project
$DEMO_BASE/o3de/scripts/o3de.sh enable-gem --gem-name WarehouseSample --project-path $PROJECT_PATH/Project
$DEMO_BASE/o3de/scripts/o3de.sh enable-gem --gem-name RosRobotSample --project-path $PROJECT_PATH/Project
```

### 6. Build the project

Next, let us the build project with the necessary elements of the O3DE engine and ROS2 Gem.

```shell
cd $PROJECT_PATH/Project
. /opt/ros/humble/setup.bash
cmake -B build/linux -G "Ninja Multi-Config" -DLY_UNITY_BUILD=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DLY_PARALLEL_LINK_JOBS=16 -DLY_STRIP_DEBUG_SYMBOLS=OFF
cmake --build build/linux --config profile --target $PROJECT_NAME.GameLauncher Editor
```

### 7. Launch Editor

Finally, previously built O3DE with preloaded ROS2 Gem can be run:

```shell
$PROJECT_PATH/Project/build/linux/bin/profile/Editor
```

## Running ROS example

Refer to `$DEMO_BASE/$PROJECT_NAME/examples/slam_navigation/README.md` for instructions.
