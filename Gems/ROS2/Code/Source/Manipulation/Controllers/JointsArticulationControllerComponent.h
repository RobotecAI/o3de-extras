/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Name/Name.h>
#include <ROS2/Manipulation/Controllers/JointsPositionControllerRequests.h>

namespace ROS2
{
    //! Handles position control commands for joints using Articulations.
    class JointsArticulationControllerComponent
        : public AZ::Component
        , public JointsPositionControllerRequestBus::Handler
    {
    public:
        JointsArticulationControllerComponent() = default;
        ~JointsArticulationControllerComponent() = default;
        AZ_COMPONENT(JointsArticulationControllerComponent, "{243E9F07-5F84-4F83-9E6D-D1DA04D7CEF9}", AZ::Component);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void Reflect(AZ::ReflectContext* context);

        // JointsPositionControllerRequestBus::Handler overrides ...
        //! @see ROS2::JointsPositionControllerRequestBus::PositionControl
        AZ::Outcome<void, AZStd::string> PositionControl(
            const AZ::Name& jointName,
            JointsManipulationRequests::JointInfo joint,
            JointsManipulationRequests::JointPosition currentPosition,
            JointsManipulationRequests::JointPosition targetPosition,
            float deltaTime) override;

    private:
        // Component overrides ...
        void Activate() override;
        void Deactivate() override;
    };
} // namespace ROS2
