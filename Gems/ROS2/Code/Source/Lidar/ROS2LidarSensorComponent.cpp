/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Atom/RPI.Public/AuxGeom/AuxGeomFeatureProcessorInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <Lidar/LidarRegistrarSystemComponent.h>
#include <Lidar/ROS2LidarSensorComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Utilities/ROS2Names.h>

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
            serializeContext->Class<ROS2LidarSensorComponent, SensorBaseType>()->Version(3)->Field(
                "lidarCore", &ROS2LidarSensorComponent::m_lidarCore);

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
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
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
    }

    ROS2LidarSensorComponent::ROS2LidarSensorComponent(
        const SensorConfiguration& sensorConfiguration, const LidarSensorConfiguration& lidarConfiguration)
        : m_lidarCore(lidarConfiguration)
    {
        m_sensorConfiguration = sensorConfiguration;
    }

    void ROS2LidarSensorComponent::Activate()
    {
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
    }

    void ROS2LidarSensorComponent::Deactivate()
    {
        StopSensor();
        m_pointCloudPublisher.reset();
        m_lidarCore.Deinit();
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

        uint32_t offsetXYZ, offsetIntensity, offsetRing, offsetAzimuth, offsetDistance, offsetReturnType, offsetTimestamp;
        offsetXYZ = 0;
        offsetIntensity = offsetXYZ + (3 * sizeof(float)) /* extra 32 padding*/ + sizeof(uint32_t) /* extra 32 padding*/;
        offsetRing = offsetIntensity + sizeof(float);
        offsetAzimuth = offsetRing + sizeof(uint16_t) /* extra 16 padding*/ + sizeof(uint16_t) /* extra 16 padding*/;
        offsetDistance = offsetAzimuth + sizeof(float);
        offsetReturnType = offsetDistance + sizeof(float);
        offsetTimestamp = offsetReturnType + sizeof(uint8_t) /* extra 56 padding*/ + sizeof(uint8_t) + sizeof(uint16_t) + sizeof(uint32_t) /* extra 56 padding*/;

        AZ_TracePrintfOnce(">>>>>>>>>>>>>>", "Offsets: \nXYZ %d,\nIntensity: %d\nRing %d\nAzimuth %d\nDistance %d\nReturn type %d\nTime stamp %d\n ",
                           offsetXYZ, offsetIntensity, offsetRing, offsetAzimuth, offsetDistance, offsetReturnType, offsetTimestamp);


//        message.point_step =
//            + 3 * sizeof(float) // XYZ
//            + sizeof(uint32_t) // padding 32
//            + sizeof(float) // intensity /\----
//            + sizeof(uint16_t) // ring /\----
//            + sizeof(uint16_t) // padding 16
//            + sizeof(float) // azimuth /\----
//            + sizeof(float) // distance /\----
//            + sizeof(uint8_t) // return type /\----
//            + sizeof(uint8_t) // padding 8
//            + sizeof(uint16_t) // padding 16
//            + sizeof(uint32_t ) // padding 32
//            + sizeof(double); // timestamp <----

        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        auto message = sensor_msgs::msg::PointCloud2();
        message.header.frame_id = ros2Frame->GetFrameID().data();
        message.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        message.height = 1;
        message.width = lastScanResults.m_points.size();
        message.point_step = 3 * sizeof(float) + sizeof(uint16_t);
        message.row_step = message.width * message.point_step;
        message.is_dense = false;
 
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

        sensor_msgs::msg::PointField pfRing;
        pfRing.name = "ring";
        pfRing.offset = 12;
        pfRing.datatype = sensor_msgs::msg::PointField::UINT16;
        pfRing.count = 1;
        message.fields.push_back(pfRing);

        size_t sizeInBytes = lastScanResults.m_points.size() * message.point_step;
        message.data.resize(sizeInBytes);
        AZ_Assert(message.row_step * message.height == sizeInBytes, "Inconsistency in the size of point cloud data");

        int offset = 0;
        for (int i = 0; i < lastScanResults.m_points.size(); ++i)
        {
            memcpy(message.data.data() + offset, &lastScanResults.m_points[i], 3 * sizeof(float));
            memcpy(message.data.data() + offset + 3 * sizeof(float), &lastScanResults.m_rings[i], sizeof(uint16_t));
            offset += message.point_step;
        }
        m_pointCloudPublisher->publish(message);
    }
} // namespace ROS2
