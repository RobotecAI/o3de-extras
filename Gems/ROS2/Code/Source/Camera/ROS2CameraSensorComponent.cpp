/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2CameraSensorComponent.h"
#include "Camera/CameraSensor.h"
#include "CameraUtilities.h"
#include "ROS2/Camera/CameraSensorRequestBus.h"
#include <ROS2/Frame/ROS2FrameComponent.h>

namespace ROS2
{
    ROS2CameraSensorComponent::ROS2CameraSensorComponent(
        const SensorConfiguration& sensorConfiguration, const CameraSensorConfiguration& cameraConfiguration)
        : m_cameraConfiguration(cameraConfiguration)
    {
        m_sensorConfiguration = sensorConfiguration;
    }

    void ROS2CameraSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        CameraSensorConfiguration::Reflect(context);

        auto* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize != nullptr)
        {
            serialize->Class<ROS2CameraSensorComponent, SensorBaseType>()->Version(5)->Field(
                "CameraSensorConfig", &ROS2CameraSensorComponent::m_cameraConfiguration);
        }
    }

    void ROS2CameraSensorComponent::Activate()
    {
        // Enable bus first, sensor can use parameters from the bus.
        CameraCalibrationRequestBus::Handler::BusConnect(GetEntityId());

        if (m_cameraConfiguration.m_colorCamera && m_cameraConfiguration.m_depthCamera)
        {
            m_cameraSensor = AZStd::make_unique<CameraRGBDSensor>(GetEntityId());
        }
        else if (m_cameraConfiguration.m_colorCamera)
        {
            m_cameraSensor = AZStd::make_unique<CameraColorSensor>(GetEntityId());
        }
        else if (m_cameraConfiguration.m_depthCamera)
        {
            m_cameraSensor = AZStd::make_unique<CameraDepthSensor>(GetEntityId());
        }

        CameraSensorRequestBus::Handler::BusConnect(GetEntityId());
        StartSensor(
            m_sensorConfiguration.m_frequency,
            [this]([[maybe_unused]] auto&&... args)
            {
                if (!m_sensorConfiguration.m_publishingEnabled)
                {
                    return;
                }
                FrequencyTick();
            });
    }

    void ROS2CameraSensorComponent::Deactivate()
    {
        StopSensor();
        m_cameraSensor.reset();
        CameraSensorRequestBus::Handler::BusDisconnect(GetEntityId());
        CameraCalibrationRequestBus::Handler::BusDisconnect(GetEntityId());
    }

    AZ::Matrix3x3 ROS2CameraSensorComponent::GetCameraMatrix() const
    {
        return CameraUtils::MakeCameraIntrinsics(
            m_cameraConfiguration.m_width, m_cameraConfiguration.m_height, m_cameraConfiguration.m_verticalFieldOfViewDeg);
    }

    int ROS2CameraSensorComponent::GetWidth() const
    {
        return m_cameraConfiguration.m_width;
    }

    int ROS2CameraSensorComponent::GetHeight() const
    {
        return m_cameraConfiguration.m_height;
    }

    float ROS2CameraSensorComponent::GetVerticalFOV() const
    {
        return m_cameraConfiguration.m_verticalFieldOfViewDeg;
    }

    bool ROS2CameraSensorComponent::IsColorCameraEnabled() const
    {
        return m_cameraConfiguration.m_colorCamera;
    }

    bool ROS2CameraSensorComponent::IsDepthCameraEnabled() const
    {
        return m_cameraConfiguration.m_depthCamera;
    }

    float ROS2CameraSensorComponent::GetNearClipDistance() const
    {
        return m_cameraConfiguration.m_nearClipDistance;
    }

    float ROS2CameraSensorComponent::GetFarClipDistance() const
    {
        return m_cameraConfiguration.m_farClipDistance;
    }

    CameraSensorConfiguration ROS2CameraSensorComponent::GetCameraSensorConfiguration() const
    {
        return m_cameraConfiguration;
    }

    CameraSensorRequests::CameraSensorPtr ROS2CameraSensorComponent::GetCameraSensor()
    {
        return m_cameraSensor;
    }

    void ROS2CameraSensorComponent::SetCameraSensor(CameraSensorRequests::CameraSensorPtr cameraSensor)
    {
        m_cameraSensor = cameraSensor;
    }

    void ROS2CameraSensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("ROS2Frame"));
    }

    void ROS2CameraSensorComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2CameraSensor"));
    }

    void ROS2CameraSensorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2CameraSensor"));
    }

    void ROS2CameraSensorComponent::FrequencyTick()
    {
        if (!m_cameraSensor)
        {
            return;
        }

        const AZ::Transform& transform = GetEntity()->GetTransform()->GetWorldTM();
        const auto timestamp = ROS2Interface::Get()->GetROSTimestamp();

        std_msgs::msg::Header messageHeader;
        messageHeader.stamp = timestamp;
        messageHeader.frame_id = GetFrameID().c_str();
        m_cameraSensor->RequestMessagePublication(transform, messageHeader);
    }
} // namespace ROS2
