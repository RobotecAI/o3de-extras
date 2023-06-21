/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Utils.h"

#include "GripperActionServer.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/TransformComponent.h>

namespace ROS2
{
    void GripperActionServer::Activate()
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();

        actionServer = rclcpp_action::create_server<GripperCommand>(ros2Node,
                                                    m_gripperActionServerName.data(),
                                                    AZStd::bind(&GripperActionServer::handleGoal, this, AZStd::placeholders::_1, AZStd::placeholders::_2),
                                                    AZStd::bind(&GripperActionServer::handleCancel, this, AZStd::placeholders::_1),
                                                    AZStd::bind(&GripperActionServer::handleAccepted, this, AZStd::placeholders::_1));
        AZ::TickBus::Handler::BusConnect();
    }

    void GripperActionServer::Deactivate()
    {
        actionServer.reset();
        AZ::TickBus::Handler::BusDisconnect();
    }

    void GripperActionServer::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GripperActionServer, AZ::Component>()
                ->Field("ActionServerName", &GripperActionServer::m_gripperActionServerName)
                ->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<GripperActionServer>("GripperActionServer", "GripperActionServer")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "GripperActionServer")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &GripperActionServer::m_gripperActionServerName,
                        "Gripper Action Server",
                        "Gripper Action Server.");
            }
        }
    }

    void GripperActionServer::OnTick(float delta, AZ::ScriptTimePoint timePoint)
    {
    }

    rclcpp_action::GoalResponse GripperActionServer::handleGoal(const rclcpp_action::GoalUUID & uuid,
                                           std::shared_ptr<const GripperCommand::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        AZ_Printf("GripperActionServer::handleGoal", "GripperActionServer::handleGoal: goal->command.position: %f\n", goal->command.position);
        AZ_Printf("GripperActionServer::handleGoal", "GripperActionServer::handleGoal: goal->command.max_effort: %f\n", goal->command.max_effort);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse GripperActionServer::handleCancel(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
    {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void GripperActionServer::handleAccepted(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
    {
        (void)goal_handle;
    }

} // namespace ROS2