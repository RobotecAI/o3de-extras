/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2ImuSensorEditorComponent.h"
#include "ROS2ImuSensorComponent.h"
#include "AzCore/Component/TransformBus.h"
#include "ROS2/Frame/ROS2FrameComponent.h"

namespace ROS2
{

    namespace Internal
    {
        const char* kImuMsgType = "sensor_msgs::msg::Imu";
    }

    ROS2ImuSensorEditorComponent::ROS2ImuSensorEditorComponent()
    {
        TopicConfiguration config;
        config.m_topic = "imu";
        config.m_type = Internal::kImuMsgType;
        const AZStd::string configName = "ImuConfig";
        auto imuConfiguration = AZStd::make_pair(configName, config);

        m_sensorConfiguration.m_frequency = 50;
        m_sensorConfiguration.m_publishersConfigurations.insert(imuConfiguration);
    }

    void ROS2ImuSensorEditorComponent::Reflect(AZ::ReflectContext* context)
    {   
        NoiseConfiguration::Reflect(context);

        auto* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<ROS2ImuSensorEditorComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(1)
                ->Field("SensorConfig", &ROS2ImuSensorEditorComponent::m_sensorConfiguration)
                ->Field("ImuNoiseConfig", &ROS2ImuSensorEditorComponent::m_noiseConfiguration);
        }
        if (AZ::EditContext* ec = serialize->GetEditContext())
        {
            ec ->Class<ROS2ImuSensorEditorComponent>("ROS2ImuSensorEditorComponent", "The ROS2ImuSensorEditorComponent is used to create a ROS2ImuSensorComponent on an entity.")
                ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2ImuSensorEditorComponent::m_sensorConfiguration, "Sensor Configuration", "Configuration for the sensor")
                ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2ImuSensorEditorComponent::m_noiseConfiguration, "IMU Noise Configuration", "Configuration for the IMU noise");
        }
    }

    void ROS2ImuSensorEditorComponent::Activate()
    {
        // AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(this->GetEntityId());
        // AzToolsFramework::Components::EditorComponentBase::Activate();
    }

    void ROS2ImuSensorEditorComponent::Deactivate()
    {
        // AzToolsFramework::Components::EditorComponentBase::Deactivate();
        // AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();
    }

    void ROS2ImuSensorEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("ROS2Frame"));
    }

    void ROS2ImuSensorEditorComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        ;  //TODO(adkrawcz): add/check incompatible services
    }

    void ROS2ImuSensorEditorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        ;  //TODO(adkrawcz): add/check provided services
    }

    void ROS2ImuSensorEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        gameEntity->CreateComponent<ROS2::ROS2ImuSensorComponent>(m_sensorConfiguration);
    }

} // namespace ROS2
