# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

set(FILES
        ../Assets/Passes/PipelineRenderToTextureROSColor.pass
        ../Assets/Passes/PipelineRenderToTextureROSDepth.pass
        ../Assets/Passes/PipelineROSColor.pass
        ../Assets/Passes/PipelineROSDepth.pass
        ../Assets/Passes/ROSPassTemplates.azasset
        Source/Camera/CameraSensor.cpp
        Source/Camera/CameraSensor.h
        Source/Camera/ROS2CameraSensorComponent.cpp
        Source/Camera/ROS2CameraSensorComponent.h
        Source/Clock/SimulationClock.cpp
        Source/Communication/QoS.cpp
        Source/Communication/TopicConfiguration.cpp
        Source/EffortSensor/ROS2EffortSensorComponent.cpp
        Source/EffortSensor/ROS2EffortSensorComponent.h
        Source/Frame/NamespaceConfiguration.cpp
        Source/Frame/ROS2FrameComponent.cpp
        Source/Frame/ROS2Transform.cpp
        Source/GNSS/GNSSFormatConversions.cpp
        Source/GNSS/GNSSFormatConversions.h
        Source/GNSS/ROS2GNSSSensorComponent.cpp
        Source/GNSS/ROS2GNSSSensorComponent.h
        Source/Imu/ROS2ImuSensorComponent.cpp
        Source/Imu/ROS2ImuSensorComponent.h
        Source/Lidar/LidarRaycaster.cpp
        Source/Lidar/LidarRaycaster.h
        Source/Lidar/LidarRegistrarSystemComponent.cpp
        Source/Lidar/LidarRegistrarSystemComponent.h
        Source/Lidar/LidarSystem.cpp
        Source/Lidar/LidarSystem.h
        Source/Lidar/LidarTemplate.cpp
        Source/Lidar/LidarTemplate.h
        Source/Lidar/LidarTemplateUtils.cpp
        Source/Lidar/LidarTemplateUtils.h
        Source/Lidar/ROS2LidarSensorComponent.cpp
        Source/Lidar/ROS2LidarSensorComponent.h
        Source/Manipulator/MotorizedJointComponent.cpp
        Source/Odometry/ROS2OdometrySensorComponent.cpp
        Source/Odometry/ROS2OdometrySensorComponent.h
        Source/RobotControl/Ackermann/AckermannSubscriptionHandler.cpp
        Source/RobotControl/Ackermann/AckermannSubscriptionHandler.h
        Source/RobotControl/ControlConfiguration.cpp
        Source/RobotControl/Controllers/AckermannController/AckermannControlComponent.cpp
        Source/RobotControl/Controllers/AckermannController/AckermannControlComponent.h
        Source/RobotControl/Controllers/RigidBodyController/RigidBodyTwistControlComponent.cpp
        Source/RobotControl/Controllers/RigidBodyController/RigidBodyTwistControlComponent.h
        Source/RobotControl/Controllers/SkidSteeringController/SkidSteeringControlComponent.cpp
        Source/RobotControl/Controllers/SkidSteeringController/SkidSteeringControlComponent.h
        Source/RobotControl/ROS2RobotControlComponent.cpp
        Source/RobotControl/ROS2RobotControlComponent.h
        Source/RobotControl/Twist/TwistSubscriptionHandler.cpp
        Source/RobotImporter/ROS2RobotImporterSystemComponent.cpp
        Source/RobotImporter/ROS2RobotImporterSystemComponent.h
        Source/ROS2ModuleInterface.h
        Source/ROS2SystemComponent.cpp
        Source/ROS2SystemComponent.h
        Source/Sensor/ROS2SensorComponent.cpp
        Source/Sensor/SensorConfiguration.cpp
        Source/Spawner/ROS2SpawnerComponent.cpp
        Source/Spawner/ROS2SpawnerComponent.h
        Source/Spawner/ROS2SpawnPointComponent.cpp
        Source/Spawner/ROS2SpawnPointComponent.h
        Source/Utilities/Controllers/PidConfiguration.cpp
        Source/Utilities/ROS2Conversions.cpp
        Source/Utilities/ROS2Names.cpp
        Source/VehicleDynamics/AxleConfiguration.cpp
        Source/VehicleDynamics/AxleConfiguration.h
        Source/VehicleDynamics/DriveModel.cpp
        Source/VehicleDynamics/DriveModel.h
        Source/VehicleDynamics/DriveModels/AckermannDriveModel.cpp
        Source/VehicleDynamics/DriveModels/AckermannDriveModel.h
        Source/VehicleDynamics/DriveModels/SkidSteeringDriveModel.cpp
        Source/VehicleDynamics/DriveModels/SkidSteeringDriveModel.h
        Source/VehicleDynamics/ManualControlEventHandler.h
        Source/VehicleDynamics/Utilities.cpp
        Source/VehicleDynamics/Utilities.h
        Source/VehicleDynamics/VehicleConfiguration.cpp
        Source/VehicleDynamics/VehicleConfiguration.h
        Source/VehicleDynamics/VehicleInputs.cpp
        Source/VehicleDynamics/VehicleInputs.h
        Source/VehicleDynamics/VehicleModelComponent.cpp
        Source/VehicleDynamics/VehicleModelComponent.h
        Source/VehicleDynamics/ModelComponents/AckermannModelComponent.cpp
        Source/VehicleDynamics/ModelComponents/AckermannModelComponent.h
        Source/VehicleDynamics/ModelComponents/SkidSteeringModelComponent.cpp
        Source/VehicleDynamics/ModelComponents/SkidSteeringModelComponent.h
        Source/VehicleDynamics/VehicleModelLimits.cpp
        Source/VehicleDynamics/VehicleModelLimits.h
        Source/VehicleDynamics/ModelLimits/AckermannModelLimits.cpp
        Source/VehicleDynamics/ModelLimits/AckermannModelLimits.h
        Source/VehicleDynamics/ModelLimits/SkidSteeringModelLimits.cpp
        Source/VehicleDynamics/ModelLimits/SkidSteeringModelLimits.h
        Source/VehicleDynamics/WheelControllerComponent.cpp
        Source/VehicleDynamics/WheelControllerComponent.h
        Source/VehicleDynamics/WheelDynamicsData.h
        )
