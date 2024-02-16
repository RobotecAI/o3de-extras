/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "CameraPublishers.h"
#include "AzCore/Component/EntityId.h"
#include "Camera/ROS2CameraSensorComponent.h"
#include "CameraConstants.h"
#include "CameraSensor.h"
#include <AzCore/Component/Entity.h>
#include <ROS2/Camera/CameraCalibrationRequestBus.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Sensor/SensorConfiguration.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    namespace Internal
    {
        using TopicConfigurations = AZStd::unordered_map<CameraPublishers::CameraChannelType, TopicConfiguration>;

        TopicConfiguration GetTopicConfiguration(const SensorConfiguration& sensorConfiguration, const AZStd::string& key)
        {
            auto ipos = sensorConfiguration.m_publishersConfigurations.find(key);
            AZ_Assert(ipos != sensorConfiguration.m_publishersConfigurations.end(), "Missing key in topic configuration!");
            return ipos != sensorConfiguration.m_publishersConfigurations.end() ? ipos->second : TopicConfiguration{};
        }

        template<typename CameraType>
        TopicConfigurations GetCameraTopicConfiguration([[maybe_unused]] const SensorConfiguration& sensorConfiguration);

        template<typename CameraType>
        TopicConfigurations GetCameraInfoTopicConfiguration([[maybe_unused]] const SensorConfiguration& sensorConfiguration);

        template<>
        TopicConfigurations GetCameraTopicConfiguration<CameraColorSensor>(const SensorConfiguration& sensorConfiguration)
        {
            return { { CameraPublishers::CameraChannelType::RGB,
                       GetTopicConfiguration(sensorConfiguration, CameraConstants::ColorImageConfig) } };
        }

        template<>
        TopicConfigurations GetCameraInfoTopicConfiguration<CameraColorSensor>(const SensorConfiguration& sensorConfiguration)
        {
            return { { CameraPublishers::CameraChannelType::RGB,
                       GetTopicConfiguration(sensorConfiguration, CameraConstants::ColorInfoConfig) } };
        }

        template<>
        TopicConfigurations GetCameraTopicConfiguration<CameraDepthSensor>(const SensorConfiguration& sensorConfiguration)
        {
            return { { CameraPublishers::CameraChannelType::DEPTH,
                       GetTopicConfiguration(sensorConfiguration, CameraConstants::DepthImageConfig) } };
        }

        template<>
        TopicConfigurations GetCameraInfoTopicConfiguration<CameraDepthSensor>(const SensorConfiguration& sensorConfiguration)
        {
            return { { CameraPublishers::CameraChannelType::DEPTH,
                       GetTopicConfiguration(sensorConfiguration, CameraConstants::DepthInfoConfig) } };
        }

        //! Helper that adds publishers based on predefined configuration.
        template<typename PublishedData>
        void AddPublishersFromConfiguration(
            const AZStd::string& cameraNamespace,
            const TopicConfigurations& configurations,
            AZStd::unordered_map<CameraPublishers::CameraChannelType, std::shared_ptr<rclcpp::Publisher<PublishedData>>>& publishers)
        {
            for (const auto& [channel, configuration] : configurations)
            {
                AZStd::string fullTopic = ROS2Names::GetNamespacedName(cameraNamespace, configuration.m_topic);
                auto ros2Node = ROS2Interface::Get()->GetNode();
                auto publisher = ros2Node->create_publisher<PublishedData>(fullTopic.data(), configuration.GetQoS());
                publishers[channel] = publisher;
            }
        }

        //! Helper that adds publishers for a camera type.
        //! @tparam CameraType type of camera sensor (eg 'CameraColorSensor').
        //! @param cameraDescription complete information about camera configuration.
        //! @param imagePublishers publishers of raw image formats (color image, depth image, ..).
        //! @param infoPublishers publishers of camera_info messages for each image topic.
        template<typename CameraType>
        void AddCameraPublishers(
            const AZ::EntityId& entityId,
            AZStd::unordered_map<CameraPublishers::CameraChannelType, CameraPublishers::ImagePublisherPtrType>& imagePublishers,
            AZStd::unordered_map<CameraPublishers::CameraChannelType, CameraPublishers::CameraInfoPublisherPtrType>& infoPublishers)
        {
            AZ::Entity* entity = AZ::Interface<AZ::ComponentApplicationRequests>::Get()->FindEntity(entityId);
            auto sensorConfiguration = entity->FindComponent<ROS2CameraSensorComponent>()->GetSensorConfiguration();

            auto* frame = entity->FindComponent<ROS2FrameComponent>();
            auto ros2Namespace = frame->GetNamespace();
            const auto cameraImagePublisherConfigs = GetCameraTopicConfiguration<CameraType>(sensorConfiguration);
            AddPublishersFromConfiguration(ros2Namespace, cameraImagePublisherConfigs, imagePublishers);
            const auto cameraInfoPublisherConfigs = GetCameraInfoTopicConfiguration<CameraType>(sensorConfiguration);
            AddPublishersFromConfiguration(ros2Namespace, cameraInfoPublisherConfigs, infoPublishers);
        }
    } // namespace Internal

    CameraPublishers::CameraPublishers(AZ::EntityId entityId)
    {
        bool colorCamera = false;
        bool depthCamera = false;
        CameraCalibrationRequestBus::EventResult(colorCamera, entityId, &CameraCalibrationRequest::IsColorCameraEnabled);
        CameraCalibrationRequestBus::EventResult(depthCamera, entityId, &CameraCalibrationRequest::IsDepthCameraEnabled);

        if (colorCamera)
        {
            Internal::AddCameraPublishers<CameraColorSensor>(entityId, m_imagePublishers, m_infoPublishers);
        }

        if (depthCamera)
        {
            Internal::AddCameraPublishers<CameraDepthSensor>(entityId, m_imagePublishers, m_infoPublishers);
        }
    }

    CameraPublishers::ImagePublisherPtrType CameraPublishers::GetImagePublisher(CameraChannelType type)
    {
        AZ_Error("GetImagePublisher", m_imagePublishers.count(type) == 1, "No publisher of this type, logic error!");
        return m_imagePublishers.at(type);
    }

    CameraPublishers::CameraInfoPublisherPtrType CameraPublishers::GetInfoPublisher(CameraChannelType type)
    {
        AZ_Error("GetInfoPublisher", m_infoPublishers.count(type) == 1, "No publisher of this type, logic error!");
        return m_infoPublishers.at(type);
    }
} // namespace ROS2
