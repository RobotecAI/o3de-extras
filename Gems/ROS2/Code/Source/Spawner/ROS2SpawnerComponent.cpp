/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SpawnerComponent.h"
#include "Spawner/ROS2SpawnerComponentController.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/Components/SimulatedBodyComponentBus.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Conversions.h>

namespace ROS2
{
    namespace Internal
    {
        bool IsEntitySimulated(const AZ::Entity* entity)
        {
            AzPhysics::SimulatedBodyHandle handle;
            // Check if the physics body is simulated
            AzPhysics::SimulatedBodyComponentRequestsBus::EventResult(
                handle, entity->GetId(), &AzPhysics::SimulatedBodyComponentRequests::GetSimulatedBodyHandle);

            return handle == AzPhysics::InvalidSimulatedBodyHandle;
        }
    } // namespace Internal

    ROS2SpawnerComponent::ROS2SpawnerComponent(const ROS2SpawnerComponentConfig& properties)
        : ROS2SpawnerComponentBase(properties)
    {
    }

    void ROS2SpawnerComponent::Activate()
    {
        ROS2SpawnerComponentBase::Activate();

        auto ros2Node = ROS2Interface::Get()->GetNode();

        const auto& ns = m_controller.GetNamespace();

        m_getSpawnablesNamesService = ros2Node->create_service<gazebo_msgs::srv::GetModelList>(
            AZStd::string::format("%s/get_available_spawnable_names", ns.data()).data(),
            [this](const GetAvailableSpawnableNamesRequest& request, const GetAvailableSpawnableNamesResponse& response)
            {
                GetAvailableSpawnableNames(request, response);
            });

        m_spawnService = ros2Node->create_service<gazebo_msgs::srv::SpawnEntity>(
            AZStd::string::format("%s/spawn_entity", ns.data()).data(),
            [this](const SpawnEntityRequest& request, const SpawnEntityResponse& response)
            {
                SpawnEntity(request, response);
            });

        m_getSpawnPointInfoService = ros2Node->create_service<gazebo_msgs::srv::GetModelState>(
            AZStd::string::format("%s/get_spawn_point_info", ns.data()).data(),
            [this](const GetSpawnPointInfoRequest& request, const GetSpawnPointInfoResponse& response)
            {
                GetSpawnPointInfo(request, response);
            });

        m_getSpawnPointsNamesService = ros2Node->create_service<gazebo_msgs::srv::GetModelList>(
            AZStd::string::format("%s/get_spawn_points_names", ns.data()).data(),
            [this](const GetSpawnPointsNamesRequest& request, const GetSpawnPointsNamesResponse& response)
            {
                GetSpawnPointsNames(request, response);
            });
    }

    void ROS2SpawnerComponent::Deactivate()
    {
        ROS2SpawnerComponentBase::Deactivate();

        m_getSpawnablesNamesService.reset();
        m_spawnService.reset();
        m_getSpawnPointInfoService.reset();
        m_getSpawnPointsNamesService.reset();
    }

    void ROS2SpawnerComponent::Reflect(AZ::ReflectContext* context)
    {
        ROS2SpawnerComponentBase::Reflect(context);

        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2SpawnerComponent, ROS2SpawnerComponentBase>()->Version(1);
        }
    }

    void ROS2SpawnerComponent::GetAvailableSpawnableNames(
        const GetAvailableSpawnableNamesRequest& request, const GetAvailableSpawnableNamesResponse& response)
    {
        for (const auto& [name, asset] : m_controller.GetSpawnables())
        {
            response->model_names.emplace_back(name.c_str());
        }
        response->success = true;
    }

    void ROS2SpawnerComponent::SpawnEntity(const SpawnEntityRequest& request, const SpawnEntityResponse& response)
    {
        AZStd::string spawnableName(request->name.c_str());
        AZStd::string spawnPointName(request->xml.c_str(), request->xml.size());
        AZStd::string spawnableCustomName(request->robot_namespace.c_str());

        auto spawnPoints = GetSpawnPoints();
        const auto& spawnables = m_controller.GetSpawnables();

        auto spawnable = spawnables.find(spawnableName);
        if (spawnable == spawnables.end())
        {
            response->success = false;
            response->status_message = "Could not find spawnable with given name: " + request->name;
            return;
        }

        AZ::Transform transform;

        if (auto it = spawnPoints.find(spawnPointName); it != spawnPoints.end())
        {
            transform = it->second.pose;
        }
        else if (spawnPointName.empty())
        {
            transform = ROS2::ROS2Conversions::FromROS2Pose(request->initial_pose);
        }
        else
        {
            response->success = false;
            response->status_message =
                "Could not find spawn point with given name: " + request->xml + ". To use initial_pose keep xml empty";
            return;
        }

        if (!m_tickets.contains(spawnableName))
        {
            // if a ticket for this spawnable was not created but the spawnable name is correct, create the ticket and then use it to
            // spawn an entity
            m_tickets.emplace(spawnable->first, AzFramework::EntitySpawnTicket(spawnable->second));
        }

        auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();

        AzFramework::SpawnAllEntitiesOptionalArgs optionalArgs;

        optionalArgs.m_preInsertionCallback = [this, transform, spawnableName, spawnableCustomName](auto id, auto view)
        {
            PreSpawn(id, view, transform, spawnableName, spawnableCustomName);
        };

        spawner->SpawnAllEntities(m_tickets.at(spawnableName), optionalArgs);

        response->success = true;
    }

    void ROS2SpawnerComponent::PreSpawn(
        AzFramework::EntitySpawnTicket::Id id [[maybe_unused]],
        AzFramework::SpawnableEntityContainerView view,
        const AZ::Transform& transform,
        const AZStd::string& spawnableName,
        const AZStd::string& customName)
    {
        if (view.empty())
        {
            return;
        }
        AZ::Entity* root = *view.begin();

        auto* transformInterface = root->FindComponent<AzFramework::TransformComponent>();
        transformInterface->SetWorldTM(transform);

        AZStd::string instanceName = customName;
        if (instanceName.empty())
        {
            instanceName = AZStd::string::format("%s_%d", spawnableName.c_str(), m_counter++);
        }
        instanceName = ROS2::ROS2Names::RosifyName(instanceName);

        bool frameComponentFound = false;
        for (AZ::Entity* entity : view)
        { // Update name for the first entity with ROS2Frame in hierarchy (left to right)
            frameComponentFound = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(entity) != nullptr;
            if (frameComponentFound)
            {
                entity->SetName(instanceName);
                return;
            }
        }

        for (AZ::Entity* entity : view)
        { // Otherwise update name for the first entity with Simulated Body in hierarchy (left to right)
            if (Internal::IsEntitySimulated(entity))
            {
                entity->SetName(instanceName);
                return;
            }
        }
        // Otherwise update name for the root of spawnable hierarchy otherwise
        root->SetName(instanceName);
    }

    void ROS2SpawnerComponent::GetSpawnPointsNames(const GetSpawnPointsNamesRequest& request, const GetSpawnPointsNamesResponse& response)
    {
        for (const auto& [name, info] : GetSpawnPoints())
        {
            response->model_names.emplace_back(name.c_str());
        }
        response->success = true;
    }

    void ROS2SpawnerComponent::GetSpawnPointInfo(const GetSpawnPointInfoRequest& request, const GetSpawnPointInfoResponse& response)
    {
        const AZStd::string_view key(request->model_name.c_str(), request->model_name.size());

        auto spawnPoints = GetSpawnPoints();
        if (auto it = spawnPoints.find(key); it != spawnPoints.end())
        {
            const auto& info = it->second;
            response->pose = ROS2Conversions::ToROS2Pose(info.pose);
            response->status_message = info.info.c_str();
            response->success = true;
        }
        else
        {
            response->status_message = "Could not find spawn point with given name: " + request->model_name;
        }
    }

    AZStd::unordered_map<AZStd::string, SpawnPointInfo> ROS2SpawnerComponent::GetSpawnPoints()
    {
        return m_controller.GetSpawnPoints();
    }
} // namespace ROS2
