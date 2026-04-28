/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Utils.h"

#include "GripperActionServerComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/TransformComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2NamesBus.h>

namespace ROS2Controllers
{
    void GripperActionServerComponent::Activate()
    {
        auto* ros2Frame = GetEntity()->FindComponent<ROS2::ROS2FrameComponent>();
        AZ_Assert(ros2Frame, "Missing Frame Component!");
        AZStd::string namespacedAction;
        ROS2::ROS2NamesRequestBus::BroadcastResult(
            namespacedAction, &ROS2::ROS2NamesRequestBus::Events::GetNamespacedName, ros2Frame->GetNamespace(), m_gripperActionServerName);
        AZ_Printf("GripperActionServerComponent", "Creating Gripper Action Server: %s\n", namespacedAction.c_str());
        m_gripperActionServer = AZStd::make_unique<GripperActionServer>(namespacedAction, GetEntityId());
        AZ::TickBus::Handler::BusConnect();
    }

    void GripperActionServerComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        m_gripperActionServer.reset();
    }

    void GripperActionServerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GripperActionServerComponent, AZ::Component>()
                ->Field("ActionServerName", &GripperActionServerComponent::m_gripperActionServerName)
                ->Field("AllowStalling", &GripperActionServerComponent::m_allowStalling)
                ->Version(2);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<GripperActionServerComponent>("GripperActionServerComponent", "Component for the gripper action server")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "GripperActionServer")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/GripperActionServerComponent.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/GripperActionServerComponent.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GripperActionServerComponent::m_gripperActionServerName,
                        "Gripper Action Server",
                        "Action name for the gripper server.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GripperActionServerComponent::m_allowStalling,
                        "Allow Stalling",
                        "If true, a stalled gripper is reported as success (e.g. grasped an object). If false, stall is reported as failure.");
            }
        }
    }

    void GripperActionServerComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
        required.push_back(AZ_CRC_CE("GripperService"));
    }
    void GripperActionServerComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("GripperServerService"));
    }
    void GripperActionServerComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("GripperServerService"));
    }

    std::shared_ptr<GripperActionServer::GripperCommand::Feedback> GripperActionServerComponent::ProduceFeedback() const
    {
        auto feedback = std::make_shared<GripperCommand::Feedback>();
        float position = 0.0f;
        float effort = 0.0f;
        GripperRequestBus::EventResult(position, GetEntityId(), &GripperRequestBus::Events::GetGripperPosition);
        GripperRequestBus::EventResult(effort, GetEntityId(), &GripperRequestBus::Events::GetGripperEffort);
        bool stalled = false;
        bool reachedGoal = false;
        GripperRequestBus::EventResult(stalled, GetEntityId(), &GripperRequestBus::Events::IsGripperNotMoving);
        GripperRequestBus::EventResult(reachedGoal, GetEntityId(), &GripperRequestBus::Events::HasGripperReachedGoal);
        feedback->position = position;
        feedback->effort = effort;
        feedback->reached_goal = reachedGoal;
        feedback->stalled = stalled;
        return feedback;
    }

    std::shared_ptr<GripperActionServer::GripperCommand::Result> GripperActionServerComponent::ProduceResult() const
    {
        auto result = std::make_shared<GripperCommand::Result>();
        float position = 0.0f;
        float effort = 0.0f;
        bool stalled = false;
        bool reachedGoal = false;
        GripperRequestBus::EventResult(position, GetEntityId(), &GripperRequestBus::Events::GetGripperPosition);
        GripperRequestBus::EventResult(effort, GetEntityId(), &GripperRequestBus::Events::GetGripperEffort);
        GripperRequestBus::EventResult(stalled, GetEntityId(), &GripperRequestBus::Events::IsGripperNotMoving);
        GripperRequestBus::EventResult(reachedGoal, GetEntityId(), &GripperRequestBus::Events::HasGripperReachedGoal);
        result->position = position;
        result->effort = effort;
        result->reached_goal = reachedGoal;
        result->stalled = stalled;
        return result;
    }

    void GripperActionServerComponent::OnTick(float delta, AZ::ScriptTimePoint timePoint)
    {
        AZ_Assert(m_gripperActionServer, "GripperActionServer::OnTick: GripperActionServer is null!");
        if (!m_gripperActionServer->IsGoalActiveState())
        {
            return;
        }
        bool isDone = false;
        bool isStalled;
        bool isCancelled = false;
        GripperRequestBus::EventResult(isDone, GetEntityId(), &GripperRequestBus::Events::HasGripperReachedGoal);
        GripperRequestBus::EventResult(isStalled, GetEntityId(), &GripperRequestBus::Events::IsGripperNotMoving);
        GripperRequestBus::EventResult(isCancelled, GetEntityId(), &GripperRequestBus::Events::HasGripperCommandBeenCancelled);
        if (isCancelled)
        {
            m_gripperActionServer->CancelGoal(ProduceResult());
            return;
        }
        if (isDone)
        {
            AZ_Printf("GripperActionServer::OnTick", "GripperActionServer::OnTick: Gripper reached goal!");
            m_gripperActionServer->GoalSuccess(ProduceResult());
            return;
        }
        if (isStalled)
        {
            if (m_allowStalling)
            {
                AZ_Printf("GripperActionServer::OnTick", "GripperActionServer::OnTick: Gripper stalled (treated as success)");
                m_gripperActionServer->GoalSuccess(ProduceResult());
            }
            else
            {
                AZ_Printf("GripperActionServer::OnTick", "GripperActionServer::OnTick: Gripper stalled (aborting)");
                m_gripperActionServer->AbortGoal(ProduceResult());
            }
            return;
        }
        m_gripperActionServer->PublishFeedback(ProduceFeedback());
    }

} // namespace ROS2Controllers
