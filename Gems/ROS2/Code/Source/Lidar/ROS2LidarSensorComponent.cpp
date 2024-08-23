/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Atom/RPI.Public/AuxGeom/AuxGeomFeatureProcessorInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <AzCore/Serialization/Json/JsonSerialization.h>
#include <Lidar/LidarRegistrarSystemComponent.h>
#include <Lidar/ROS2LidarSensorComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Utilities/ROS2Names.h>

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

namespace ROS2
{
    namespace
    {
        const char* PointCloudType = "sensor_msgs::msg::PointCloud2";
    }

    void ROS2LidarSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2LidarSensorComponent, SensorBaseType>()
                ->Version(3)
                ->Field("lidarCore", &ROS2LidarSensorComponent::m_lidarCore)
                ->Field("ParametersSetterTopicConfiguration", &ROS2LidarSensorComponent::m_parametersSetterConfigurationTopic)
                ->Field("ParametersGetterTopicConfiguration", &ROS2LidarSensorComponent::m_parametersGetterConfigurationTopic);

            if (auto* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<ROS2LidarSensorComponent>("ROS2 Lidar Sensor", "Lidar sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2LidarSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2LidarSensor.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &ROS2LidarSensorComponent::m_lidarCore,
                        "Lidar configuration",
                        "Lidar configuration")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2LidarSensorComponent::m_parametersSetterConfigurationTopic,
                        "Parameters setter topic",
                        "Topic to receive parameters for the sensor")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2LidarSensorComponent::m_parametersGetterConfigurationTopic,
                        "Parameters getter topic",
                        "Topic to publish parameters for the sensor");
            }
        }
    }

    void ROS2LidarSensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    ROS2LidarSensorComponent::ROS2LidarSensorComponent()
        : m_lidarCore(LidarTemplateUtils::Get3DModels())
    {
        TopicConfiguration pc;
        AZStd::string type = PointCloudType;
        pc.m_type = type;
        pc.m_topic = "pc";
        m_sensorConfiguration.m_frequency = 10.f;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, pc));

        m_parametersSetterConfigurationTopic.m_topic = "SetLidarParameters";
        m_parametersSetterConfigurationTopic.m_type = "std_msgs/msg/String";

        m_parametersGetterConfigurationTopic.m_topic = "GetLidarParameters";
        m_parametersGetterConfigurationTopic.m_type = "std_msgs/msg/String";
    }

    ROS2LidarSensorComponent::ROS2LidarSensorComponent(
        const SensorConfiguration& sensorConfiguration, const LidarSensorConfiguration& lidarConfiguration)
        : m_lidarCore(lidarConfiguration)
    {
        m_sensorConfiguration = sensorConfiguration;
    }

    void ROS2LidarSensorComponent::Activate()
    {
        ROS2SensorComponentBase::Activate();
        m_lidarCore.Init(GetEntityId());

        m_lidarRaycasterId = m_lidarCore.GetLidarRaycasterId();
        m_canRaycasterPublish = false;
        if (m_lidarCore.m_lidarConfiguration.m_lidarSystemFeatures & LidarSystemFeatures::PointcloudPublishing)
        {
            LidarRaycasterRequestBus::EventResult(
                m_canRaycasterPublish, m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::CanHandlePublishing);
        }

        if (m_canRaycasterPublish)
        {
            const TopicConfiguration& publisherConfig = m_sensorConfiguration.m_publishersConfigurations[PointCloudType];
            auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());

            LidarRaycasterRequestBus::Event(
                m_lidarRaycasterId,
                &LidarRaycasterRequestBus::Events::ConfigurePointCloudPublisher,
                ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic),
                ros2Frame->GetFrameID().data(),
                publisherConfig.GetQoS());
        }
        else
        {
            auto ros2Node = ROS2Interface::Get()->GetNode();
            AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for lidar sensor");

            const TopicConfiguration& publisherConfig = m_sensorConfiguration.m_publishersConfigurations[PointCloudType];
            AZStd::string fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
            m_pointCloudPublisher = ros2Node->create_publisher<sensor_msgs::msg::PointCloud2>(fullTopic.data(), publisherConfig.GetQoS());
        }

        StartSensor(
            m_sensorConfiguration.m_frequency,
            [this]([[maybe_unused]] auto&&... args)
            {
                if (!m_sensorConfiguration.m_publishingEnabled)
                {
                    return;
                }
                FrequencyTick();
            },
            [this]([[maybe_unused]] auto&&... args)
            {
                if (!m_sensorConfiguration.m_visualize)
                {
                    return;
                }
                m_lidarCore.VisualizeResults();
            });

        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        AZ_Assert(ros2Node, "ROS 2 node is not initialized");

        m_parametersConfigurationTopicSubscription = ros2Node->create_subscription<std_msgs::msg::String>(
            ROS2Names::GetNamespacedName(GetNamespace(), m_parametersSetterConfigurationTopic.m_topic).c_str(),
            m_parametersSetterConfigurationTopic.GetQoS(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                // Copy the message to a string to avoid lifetime issues.
                AZStd::string msgString = msg->data.c_str();
                SetConfigurationFormJsonString(msgString);
            });

        m_parametersGetConfigurationTopicPublisher = ros2Node->create_publisher<std_msgs::msg::String>(
            ROS2Names::GetNamespacedName(GetNamespace(), m_parametersGetterConfigurationTopic.m_topic).c_str(),
            m_parametersGetterConfigurationTopic.GetQoS());

        PublishConfiguration();
    }

    void ROS2LidarSensorComponent::Deactivate()
    {
        m_parametersConfigurationTopicSubscription.reset();
        m_parametersGetConfigurationTopicPublisher.reset();

        StopSensor();
        m_pointCloudPublisher.reset();
        m_lidarCore.Deinit();
        ROS2SensorComponentBase::Deactivate();
    }

    void ROS2LidarSensorComponent::FrequencyTick()
    {
        auto entityTransform = GetEntity()->FindComponent<AzFramework::TransformComponent>();

        if (m_canRaycasterPublish)
        {
            const builtin_interfaces::msg::Time timestamp = ROS2Interface::Get()->GetROSTimestamp();
            LidarRaycasterRequestBus::Event(
                m_lidarRaycasterId,
                &LidarRaycasterRequestBus::Events::UpdatePublisherTimestamp,
                aznumeric_cast<AZ::u64>(timestamp.sec) * aznumeric_cast<AZ::u64>(1.0e9f) + timestamp.nanosec);
        }

        auto lastScanResults = m_lidarCore.PerformRaycast();

        if (m_canRaycasterPublish)
        { // Skip publishing when it can be handled by the raycaster.
            return;
        }

        const auto inverseLidarTM = entityTransform->GetWorldTM().GetInverse();
        for (auto& point : lastScanResults.m_points)
        {
            point = inverseLidarTM.TransformPoint(point);
        }

        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        auto message = sensor_msgs::msg::PointCloud2();
        message.header.frame_id = ros2Frame->GetFrameID().data();
        message.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        message.height = 1;
        message.width = lastScanResults.m_points.size();
        message.point_step = sizeof(AZ::Vector3);
        message.row_step = message.width * message.point_step;

        AZStd::array<const char*, 3> point_field_names = { "x", "y", "z" };
        for (int i = 0; i < point_field_names.size(); i++)
        {
            sensor_msgs::msg::PointField pf;
            pf.name = point_field_names[i];
            pf.offset = i * 4;
            pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
            pf.count = 1;
            message.fields.push_back(pf);
        }

        size_t sizeInBytes = lastScanResults.m_points.size() * sizeof(AZ::Vector3);
        message.data.resize(sizeInBytes);
        AZ_Assert(message.row_step * message.height == sizeInBytes, "Inconsistency in the size of point cloud data");
        memcpy(message.data.data(), lastScanResults.m_points.data(), sizeInBytes);
        m_pointCloudPublisher->publish(message);
    }

    void ROS2LidarSensorComponent::SetConfigurationFormJsonString(AZStd::string configString)
    {
        rapidjson::Document document;
        document.Parse(configString.data());

        Deactivate();
        AZ::JsonSerialization::Load(this, ROS2LidarSensorComponent::RTTI_Type(), document);
        Activate();
    }

    void ROS2LidarSensorComponent::PublishConfiguration()
    {
        rapidjson::Document document;
        document.SetObject();
        AZ::JsonSerializerSettings settings;
        settings.m_keepDefaults = true;

        AZ::JsonSerialization::Store(document, document.GetAllocator(), this, nullptr, ROS2LidarSensorComponent::RTTI_Type(), settings);

        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        document.Accept(writer);

        std_msgs::msg::String msg;
        msg.data = buffer.GetString();

        m_parametersGetConfigurationTopicPublisher->publish(msg);
    }
} // namespace ROS2
