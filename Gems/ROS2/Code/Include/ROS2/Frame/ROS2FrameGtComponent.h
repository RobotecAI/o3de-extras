/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <AzFramework/Components/TransformComponent.h>
#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Frame/ROS2Transform.h>
#include <ROS2/ROS2GemUtilities.h>

namespace ROS2
{
    //! This component marks an interesting reference frame for ROS2 ecosystem.
    //! It serves as sensor data frame of reference and is responsible, through ROS2Transform, for publishing
    //! ros2 static and dynamic transforms (/tf_static, /tf). It also facilitates namespace handling.
    //! An entity can only have a single ROS2Frame on each level. Many ROS2 Components require this component.
    //! @note A robot should have this component on every level of entity hierarchy (for each joint, fixed or dynamic)
    class ROS2FrameGtComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2FrameGtComponent, "{81B6F16A-76DD-461B-ACF6-E5B1F03C4F9C}");

        ROS2FrameGtComponent();
        //! Initialize to a specific frame id
        explicit ROS2FrameGtComponent(const AZStd::string& frameId);

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

        static void Reflect(AZ::ReflectContext* context);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        //! Get a frame id, which is needed for any ROS2 message with a Header
        //! @return Frame id which includes the namespace, ready to send in a ROS2 message
        AZStd::string GetFrameID() const;

        //! Get a namespace, which should be used for any publisher or subscriber in the same entity.
        //! @return A complete namespace (including parent namespaces)
        AZStd::string GetNamespace() const;

        //! Get a transform between this frame and the next frame up in hierarchy.
        //! @return If the parent frame is found, return a Transform between this frame and the parent.
        //! Otherwise, return a global Transform.
        //! @note Parent frame is not the same as parent Transform: there could be many Transforms in between without ROS2Frame components.
        AZ::Transform GetFrameTransform() const;

        //! Global frame name in ros2 ecosystem.
        //! @return The name of the global frame with namespace attached. It is typically "odom", "map", "world".
        AZStd::string GetGlobalFrameName() const;

    private:
        //////////////////////////////////////////////////////////////////////////
        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        //////////////////////////////////////////////////////////////////////////

        NamespaceConfiguration m_namespaceConfiguration;
        AZStd::string m_referenceFrameName = "map_gt";
        AZStd::string m_frameName = "sensor_frame_gt";

        AZStd::unique_ptr<ROS2Transform> m_ros2Transform;
        AZ::Transform m_referenceFromWorld;
    };
} // namespace ROS2
