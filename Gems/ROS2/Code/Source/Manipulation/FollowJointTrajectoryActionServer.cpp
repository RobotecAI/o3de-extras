/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FollowJointTrajectoryActionServer.h"
#include <AzCore/std/functional.h>
#include <ROS2/Manipulation/ManipulatorTrajectoryRequestBus.h>
#include <ROS2/ROS2Bus.h>

namespace ROS2
{
    void FollowJointTrajectoryActionServer::FollowJointTrajectoryActionServer(const AZStd::string& actionName, const AZ::EntityId& entityId)
        : m_entityId(entityId)
    {
        m_actionServer = rclcpp_action::create_server<FollowJointTrajectory>(
            ROS2Interface::Get()->GetNode(),
            actionName,
            AZStd::bind(&FollowJointTrajectoryActionServer::GoalReceivedCallback, this, AZStd::placeholders::_1, AZStd::placeholders::_2),
            AZStd::bind(&FollowJointTrajectoryActionServer::GoalCancelledCallback, this, AZStd::placeholders::_1),
            AZStd::bind(&FollowJointTrajectoryActionServer::GoalAcceptedCallback, this, AZStd::placeholders::_1));
    }

    ManipulatorActionStatus FollowJointTrajectoryActionServer::GeGoalStatus() const
    {
        return m_goalStatus;
    }

    void FollowJointTrajectoryActionServer::CancelGoal(std::shared_ptr<FollowJointTrajectory::Result> result)
    {
        AZ_Assert(m_goalHandle, "Invalid goal handle!");
        if (m_goalHandle && m_goalHandle->is_executing())
        {
            AZ_TracePrintf("FollowJointTrajectoryActionServe`r", "Cancelling goal);
            m_goalHandle->cancelled(result);
        }
    }

    void FollowJointTrajectoryActionServer::GoalSuccess(std::shared_ptr<FollowJointTrajectory::Result> result)
    {
        AZ_Assert(m_goalHandle, "Invalid goal handle!");
        if (m_goalHandle && m_goalHandle->is_executing())
        {
            AZ_TracePrintf("FollowJointTrajectoryActionServer", "Goal succeeded");
            m_goalHandle->succeed(result);
            m_goalStatus = ManipulatorActionStatus::Succeeded;
        }
    }

    void FollowJointTrajectoryActionServer::PublishFeedback(std::shared_ptr<FollowJointTrajectory::Feedback> feedback)
    {
        AZ_Assert(m_goalHandle, "Invalid goal handle!");
        if (m_goalHandle && m_goalHandle->is_executing())
        {
            m_goalHandle->publish_feedback(feedback);
        }
    }

    bool FollowJointTrajectoryActionServer::IsReadyForExecution() const;
    {
        // Has a goal handle yet - can be accepted.
        if (!m_goalHandle)
        {
            return true;
        }

        // Status is pending (active but not_executing).
        return m_goalHandle->is_active && !m_goalHandle->is_executing();
    }

    bool FollowJointTrajectoryActionServer::IsExecuting() const
    {
        return m_goalHandle && m_goalHandle->is_executing();
    }

    rclcpp_action::GoalResponse FollowJointTrajectoryActionServer::GoalReceivedCallback(
        [[maybe_unused]] const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    { // Accept each received goal unless other goal is active (no deferring/queuing)
        if (!IsReadyForExecution())
        {
            AZ_TracePrintf("FollowJointTrajectoryActionServer", "Goal rejected: server is not ready for execution!");
            return rclcpp_action::GoalResponse::REJECT;
        }

        AZ::Outcome<void, AZ::String> executionOrderOutcome;
        ManipulatorTrajectoryRequestBus::EventResult(
            executionOrderOutcome, m_entityId, &ManipulatorTrajectoryRequests::StartTrajectoryGoal, goal);

        if (!executionOrderOutcome)
        {
            AZ_TracePrintf("FollowJointTrajectoryActionServer", "Execution not be accepted: %s", executionOrderOutcome.GetError().c_str());
            // TODO - for mismatched joints and other cases, the correct way: accept the goal and then cancel / abort with the Result.
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse FollowJointTrajectoryActionServer::GoalCancelledCallback(
        [[maybe_unused]] const std::shared_ptr<GoalHandle> goalHandle)
    { // Accept each cancel attempt
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_string = "User Cancelled";
        result->error_code = FollowJointTrajectory::SUCCESSFUL;

        AZ::Outcome<void, AZ::String> cancelOutcome;
        ManipulatorTrajectoryRequestBus::EventResult(
            cancelOutcome, m_entityId, &ManipulatorTrajectoryRequests::CancelTrajectoryGoal, result);

        if (!cancelOutcome)
        { // This will not happen in simulation unless intentionally done for behavior validation
            AZ_TracePrintf(
                "FollowJointTrajectoryActionServer", "Cancelling could not be accepted: %s", executionOrderOutcome.GetError().c_str());
            return rclcpp_action::CancelResonse::REJECT;
        }

        m_goalStatus = ManipulatorActionStatus::Cancelled;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void FollowJointTrajectoryActionServer::GoalAcceptedCallback(const std::shared_ptr<GoalHandle> goalHandle)
    {
        AZ_TracePrintf("FollowJointTrajectoryActionServer", "Goal accepted");
        m_goalHandle = goalHandle;
        m_goalHandle->execute();
        m_goalStatus = ManipulatorActionStatus::Executing;
    }
} // namespace ROS2
