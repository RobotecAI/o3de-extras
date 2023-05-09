/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzToolsFramework/API/ComponentEntitySelectionBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Frame/ROS2Transform.h>
#include <ROS2/Sensor/SensorConfiguration.h>
#include <ROS2/Imu/NoiseConfiguration.h>

namespace ROS2
{
    //! ROS2 Camera Editor sensor component class
    //! Allows turning an entity into a camera sensor in Editor
    //! Component draws camera frustrum in the Editor
    class ROS2ImuSensorEditorComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , protected AzFramework::EntityDebugDisplayEventBus::Handler
    {
    public:
        ROS2ImuSensorEditorComponent();
        ~ROS2ImuSensorEditorComponent() override = default;
        AZ_EDITOR_COMPONENT(ROS2ImuSensorEditorComponent, "{FCD3A871-2C8A-423B-8424-141D1488015C}");
        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);

        void Activate() override;
        void Deactivate() override;

        // AzToolsFramework::Components::EditorComponentBase overrides
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        SensorConfiguration m_sensorConfiguration;
        NoiseConfiguration m_imuNoiseConfiguration;
    };
} // namespace ROS2