/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <Atom/RPI.Public/AuxGeom/AuxGeomDraw.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/Lidar/LidarRegistrarBus.h>
#include <ROS2/Lidar/LidarSystemBus.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_msgs/msg/string.hpp>

#include "LidarCore.h"
#include "LidarRaycaster.h"
#include "LidarSensorConfiguration.h"

namespace ROS2
{
    //! Lidar sensor Component.
    //! Lidars (Light Detection and Ranging) emit laser light and measure it after reflection.
    //! Lidar Component allows customization of lidar type and behavior and encapsulates both simulation
    //! and data publishing. It requires ROS2FrameComponent.
    class ROS2LidarSensorComponent : public ROS2SensorComponentBase<TickBasedSource>
    {
    public:
        AZ_COMPONENT(ROS2LidarSensorComponent, ROS2LidarSensorComponentTypeId, SensorBaseType);
        ROS2LidarSensorComponent();
        ROS2LidarSensorComponent(const SensorConfiguration& sensorConfiguration, const LidarSensorConfiguration& lidarConfiguration);
        ~ROS2LidarSensorComponent() = default;
        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        //////////////////////////////////////////////////////////////////////////
        void FrequencyTick();

        // Deactivates and reactivates the component to apply changes loaded using JsonSerialization.
        void SetConfigurationFormJsonString(AZStd::string configString);

        // Publishes the current configuration to the topic.
        void PublishConfiguration();

        bool m_canRaycasterPublish = false;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_pointCloudPublisher;

        LidarCore m_lidarCore;

        LidarId m_lidarRaycasterId;

        ROS2::TopicConfiguration m_parametersSetterConfigurationTopic;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_parametersConfigurationTopicSubscription;

        ROS2::TopicConfiguration m_parametersGetterConfigurationTopic;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_parametersGetConfigurationTopicPublisher;
    };
} // namespace ROS2
