/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/Manipulation/JointsManipulationRequests.h>
#include <ROS2/RobotControl/ControlSubscriptionHandler.h>
#include <sensor_msgs/msg/joint_state.hpp>

namespace ROS2
{
    //! This component implements finger gripper functionality.
    class JointsStateFollowerComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        JointsStateFollowerComponent() = default;
        JointsStateFollowerComponent(const TopicConfiguration& topicConfiguration, const AZStd::vector<AZStd::string>& jointNames);
        ~JointsStateFollowerComponent() = default;
        AZ_COMPONENT(JointsStateFollowerComponent, "{185317ce-3b38-4e36-919e-c7c1758eb662}", AZ::Component);

        // AZ::Component overrides...
        void Activate() override;
        void Deactivate() override;

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        static void Reflect(AZ::ReflectContext* context);

    private:
        // AZ::TickBus::Handler overrides...
        void OnTick(float delta, AZ::ScriptTimePoint timePoint) override;

        void ProcessPositionControlMessage(const sensor_msgs::msg::JointState& message);

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_jointStates;
        ROS2::TopicConfiguration m_topicConfiguration; //!< Configuration of the subscribed topic.
        AZStd::vector<AZStd::string> m_jointNames; //!< Ordered list of joint names that can be modified via subscriber
        AZ::EntityId m_rootOfArticulation; //!< The root of the articulation chain
    };
} // namespace ROS2
