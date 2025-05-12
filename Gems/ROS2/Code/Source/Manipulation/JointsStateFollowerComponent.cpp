/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsStateFollowerComponent.h"
#include "AzCore/Debug/Trace.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <ROS2/Manipulation/JointsManipulationRequests.h>
#include <ROS2/ROS2GemUtilities.h>
#include <Utilities/ArticulationsUtilities.h>

namespace ROS2
{

    JointsStateFollowerComponent::JointsStateFollowerComponent(
        const TopicConfiguration& topicConfiguration, const AZStd::vector<AZStd::string>& jointNames)
        : m_topicConfiguration(topicConfiguration)
        , m_jointNames(jointNames)
    {
    }

    void JointsStateFollowerComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
        required.push_back(AZ_CRC_CE("JointsControllerService"));
    }

    void JointsStateFollowerComponent::Activate()
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        // create input subscriber
        m_jointStates = ros2Node->create_subscription<sensor_msgs::msg::JointState>(
            m_topicConfiguration.m_topic.data(),
            m_topicConfiguration.GetQoS(),
            std::bind(&JointsStateFollowerComponent::ProcessPositionControlMessage, this, std::placeholders::_1));

        AZ::TickBus::Handler::BusConnect();
    }

    void JointsStateFollowerComponent::Deactivate()
    {
        if (m_jointStates)
        {
            m_jointStates.reset();
        }

        AZ::TickBus::Handler::BusDisconnect();
    }

    void JointsStateFollowerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointsStateFollowerComponent, AZ::Component>()
                ->Version(0)
                ->Field("topicConfiguration", &JointsStateFollowerComponent::m_topicConfiguration)
                ->Field("jointNames", &JointsStateFollowerComponent::m_jointNames);
        }
    }

    void JointsStateFollowerComponent::OnTick([[maybe_unused]] float delta, [[maybe_unused]] AZ::ScriptTimePoint timePoint)
    {
        if (!m_rootOfArticulation.IsValid())
        {
            m_rootOfArticulation = Utils::GetRootOfArticulation(GetEntityId());
            AZ_Warning(
                "JointsStateFollowerComponent",
                m_rootOfArticulation.IsValid(),
                "Entity %s is not part of an articulation.",
                GetEntity()->GetName().c_str());

            AZ::TickBus::Handler::BusDisconnect();
        }
    }

    void JointsStateFollowerComponent::ProcessPositionControlMessage(const sensor_msgs::msg::JointState& message)
    {
        if (message.position.size() != m_jointNames.size())
        {
            AZ_Error(
                "JointsStateFollowerComponent",
                false,
                "PositionController: command size %d does not match the number of joints %d",
                message.position.size(),
                m_jointNames.size());
            return;
        }

        for (int i = 0; i < message.position.size(); i++)
        {
            AZ::Outcome<void, AZStd::string> result;
            JointsManipulationRequestBus::EventResult(
                result, m_rootOfArticulation, &JointsManipulationRequests::MoveJointToPosition, m_jointNames[i], message.position[i]);
            if (!result.IsSuccess())
            {
                AZ_Warning("DEBUG", false, "Send pose %f",message.position[i]);
                AZ_Error(
                    "JointsStateFollowerComponent",
                    result,
                    "PositionController: failed for joint %s and command %d: ",
                    m_jointNames[i].c_str(),
                    message.position[i],
                    result.GetError().c_str());
            }
        }
    }

} // namespace ROS2
