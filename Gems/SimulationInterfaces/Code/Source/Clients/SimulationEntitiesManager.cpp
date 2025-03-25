/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationEntitiesManager.h"

#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Console/IConsole.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <AzFramework/Physics/Components/SimulatedBodyComponentBus.h>
#include <AzCore/std/string/regex.h>
#include <AzCore/Component/TransformBus.h>

namespace SimulationInterfacesCommands
{
    using namespace SimulationInterfaces;
    static void simulationinterfaces_GetEntities(const AZ::ConsoleCommandContainer& arguments)
    {
        AZ_UNUSED(arguments);
        // Put your command handler code here
        AZ_Printf("SimulationInterfaces", "simulationinterfaces_GetEntities\n");
        AZStd::vector<AZStd::string> entities;
        SimulationInterfacesRequestBus::BroadcastResult(entities, &SimulationInterfacesRequestBus::Events::GetEntities, EntityFilter());
        AZ_Printf("SimulationInterfaces", "Number of simulation entities: %d\n", entities.size());
        for (const auto& entity : entities)
        {
            AZ_Printf("SimulationInterfaces", "      - %s\n", entity.c_str());
        }
    }

    static void simulationinterfaces_GetEntitiesSphere(const AZ::ConsoleCommandContainer& arguments)
    {
        float sphereShape = 10.f;
        AZ::Vector3 position = AZ::Vector3::CreateZero();
        sphereShape = arguments.empty() ? 10.f: (AZStd::stof(AZStd::string(arguments[0])));
        position.SetX( arguments.size()>1 ? (AZStd::stof(AZStd::string(arguments[1]))):0.f);
        position.SetY( arguments.size()>2 ? (AZStd::stof(AZStd::string(arguments[2]))):0.f);
        position.SetZ( arguments.size()>3 ? (AZStd::stof(AZStd::string(arguments[3]))):0.f);

        AZ_Printf("SimulationInterfaces", "simulationinterfaces_GetEntities in radius %f \n", sphereShape);
        AZ_Printf("SimulationInterfaces", "position %f %f %f \n", position.GetX(), position.GetY(), position.GetZ());
        EntityFilter filter;
        filter.m_bounds_shape = AZStd::make_shared<Physics::SphereShapeConfiguration>(sphereShape);

        AZStd::vector<AZStd::string> entities;
        SimulationInterfacesRequestBus::BroadcastResult(entities, &SimulationInterfacesRequestBus::Events::GetEntities, filter);
        AZ_Printf("SimulationInterfaces", "Number of simulation entities: %d\n", entities.size());
        for (const auto& entity : entities)
        {
            AZ_Printf("SimulationInterfaces", "      - %s\n", entity.c_str());
        }
    }

    static void simulationinterfaces_GetEntityState(const AZ::ConsoleCommandContainer& arguments)
    {
        if (arguments.empty())
        {
            AZ_Printf("SimulationInterfaces", "simulationinterfaces_GetEntityState requires entity name\n");
            return;
        }
        const AZStd::string entityName = arguments[0];
        AZ_Printf("SimulationInterfaces", "simulationinterfaces_GetEntityState %s\n", entityName.c_str());
        EntityState entityState;
        SimulationInterfacesRequestBus::BroadcastResult(entityState, &SimulationInterfacesRequestBus::Events::GetEntityState, entityName);
        AZ_Printf("SimulationInterfaces", "Entity %s\n", entityName.c_str());
        AZ_Printf("SimulationInterfaces", "Pose %f %f %f\n", entityState.m_pose.GetTranslation().GetX(), entityState.m_pose.GetTranslation().GetY(), entityState.m_pose.GetTranslation().GetZ());
        AZ_Printf("SimulationInterfaces", "Rotation (quaternion) %f %f %f %f\n", entityState.m_pose.GetRotation().GetX(), entityState.m_pose.GetRotation().GetY(), entityState.m_pose.GetRotation().GetZ(), entityState.m_pose.GetRotation().GetW());
        const AZ::Vector3 euler = entityState.m_pose.GetRotation().GetEulerDegrees();
        AZ_Printf("SimulationInterfaces", "Rotation (euler) %f %f %f\n", euler.GetX(), euler.GetY(), euler.GetZ());
        AZ_Printf("SimulationInterfaces", "Twist Linear %f %f %f\n", entityState.m_twist_linear.GetX(), entityState.m_twist_linear.GetY(), entityState.m_twist_linear.GetZ());
        AZ_Printf("SimulationInterfaces", "Twist Angular %f %f %f\n", entityState.m_twist_angular.GetX(), entityState.m_twist_angular.GetY(), entityState.m_twist_angular.GetZ());

    }

    static void simulationinterfaces_SetStateXYZ(const AZ::ConsoleCommandContainer& arguments)
    {
        if (arguments.empty())
        {
            AZ_Printf("SimulationInterfaces", "simulationinterfaces_GetEntityState requires entity name\n");
            return;
        }
        const AZStd::string entityName = arguments[0];
        AZ::Vector3 position = AZ::Vector3::CreateZero();
        position.SetX( arguments.size()>1 ? (AZStd::stof(AZStd::string(arguments[1]))):0.f);
        position.SetY( arguments.size()>2 ? (AZStd::stof(AZStd::string(arguments[2]))):0.f);
        position.SetZ( arguments.size()>3 ? (AZStd::stof(AZStd::string(arguments[3]))):0.f);
        EntityState entityState {};
        entityState.m_pose = AZ::Transform::CreateIdentity();
        entityState.m_pose.SetTranslation(position);
        bool isOk = false;
        SimulationInterfacesRequestBus::BroadcastResult(isOk, &SimulationInterfacesRequestBus::Events::SetEntityState, entityName, entityState);
        if (isOk)
        {
            AZ_Printf("SimulationInterfaces", "Entity %s state set\n", entityName.c_str());
        }
        else
        {
            AZ_Printf("SimulationInterfaces", "Entity %s state NOT set\n", entityName.c_str());
        }

    }


    AZ_CONSOLEFREEFUNC(
        simulationinterfaces_GetEntities, AZ::ConsoleFunctorFlags::DontReplicate, "Get all simulated entities in the scene.");
    AZ_CONSOLEFREEFUNC(
        simulationinterfaces_GetEntitiesSphere, AZ::ConsoleFunctorFlags::DontReplicate, "Get all simulated entities in the radius.");
    AZ_CONSOLEFREEFUNC(
        simulationinterfaces_GetEntityState, AZ::ConsoleFunctorFlags::DontReplicate, "Get state of the entity.");
    AZ_CONSOLEFREEFUNC(
        simulationinterfaces_SetStateXYZ, AZ::ConsoleFunctorFlags::DontReplicate, "Set state of the entity.");
} // namespace SimulationInterfacesCommands
namespace SimulationInterfaces
{

    void SetRigidBodyVelocities(AzPhysics::RigidBody* rigidBody, const EntityState& state)
    {
        if (!state.m_twist_angular.IsClose(AZ::Vector3::CreateZero(), AZ::Constants::FloatEpsilon))
        {
            // get transform
            AZ::Vector3 angularVelWorld = rigidBody->GetTransform().TransformVector(state.m_twist_angular);
            rigidBody->SetAngularVelocity(angularVelWorld);
        }

        if (!state.m_twist_linear.IsClose(AZ::Vector3::CreateZero(), AZ::Constants::FloatEpsilon))
        {
            // get transform
            AZ::Vector3 linearVelWorld = rigidBody->GetTransform().TransformVector(state.m_twist_linear);
            rigidBody->SetAngularVelocity(linearVelWorld);
        }
    }

    AZ_COMPONENT_IMPL(SimulationEntitiesManager, "SimulationEntitiesManager", SimulationInterfacesSystemComponentTypeId);

    void SimulationEntitiesManager::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SimulationEntitiesManager, AZ::Component>()->Version(0);
        }
    }

    void SimulationEntitiesManager::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("SimulationInterfacesService"));
    }

    void SimulationEntitiesManager::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("SimulationInterfacesService"));
    }

    void SimulationEntitiesManager::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("PhysicsService"));
    }

    void SimulationEntitiesManager::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    SimulationEntitiesManager::SimulationEntitiesManager()
    {
        if (SimulationInterfacesInterface::Get() == nullptr)
        {
            SimulationInterfacesInterface::Register(this);
        }
    }

    SimulationEntitiesManager::~SimulationEntitiesManager()
    {
        if (SimulationInterfacesInterface::Get() == this)
        {
            SimulationInterfacesInterface::Unregister(this);
        }
    }

    void SimulationEntitiesManager::Init()
    {
    }

    AzPhysics::Scene* GetSceneHelper(AzPhysics::SceneHandle sceneHandle)
    {
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "Physics system is not available.");
        AzPhysics::Scene* scene = physicsSystem->GetScene(sceneHandle);
        return scene;
    }
    void SimulationEntitiesManager::Activate()
    {
        m_simulationBodyAddedHandler = AzPhysics::SceneEvents::OnSimulationBodyAdded::Handler(
            [this](AzPhysics::SceneHandle sceneHandle, AzPhysics::SimulatedBodyHandle bodyHandle)
            {
                auto* scene = GetSceneHelper(sceneHandle);
                if (scene == nullptr)
                {
                    return;
                }
                auto* body = scene->GetSimulatedBodyFromHandle(bodyHandle);
                AZ_Assert(body, "Simulated body is not available.");
                auto *rigidBody = azdynamic_cast<AzPhysics::RigidBody*>(body);
                if (rigidBody != nullptr)
                {
                    auto shapeCount = rigidBody->GetShapeCount();
                    AZ_Warning("SimulationInterfaces", shapeCount > 0, "Entity %s has no collider shapes, it won't be available by bound search", rigidBody->GetEntityId().ToString().c_str());
                }
                const AZ::EntityId entityId = body->GetEntityId();
                // register simulated entity
                this->AddSimulatedEntity(entityId);
            });
        m_simulationBodyRemovedHandler = AzPhysics::SceneEvents::OnSimulationBodyRemoved::Handler(
            [this](AzPhysics::SceneHandle sceneHandle, AzPhysics::SimulatedBodyHandle bodyHandle)
            {
                auto* scene = GetSceneHelper(sceneHandle);
                if (scene == nullptr)
                {
                    return;
                }
                const auto* body = scene->GetSimulatedBodyFromHandle(bodyHandle);
                AZ_Assert(body, "Simulated body is not available.");
                const AZ::EntityId entityId = body->GetEntityId();
                // remove simulated entity
                this->RemoveSimulatedEntity(entityId);
            });

        m_sceneAddedHandler = AzPhysics::SystemEvents::OnSceneAddedEvent::Handler(
            [this](AzPhysics::SceneHandle sceneHandle)
            {
                AZ_Warning("SimulationInterfaces", m_physicsScenesHandle == AzPhysics::InvalidSceneHandle, "Hmm, we already have a scene");
                auto* scene = GetSceneHelper(sceneHandle);
                AZ_Assert(scene, "Scene is not available.");
                if (scene == nullptr)
                {
                    return;
                }
                scene->RegisterSimulationBodyAddedHandler(m_simulationBodyAddedHandler);
                scene->RegisterSimulationBodyRemovedHandler(m_simulationBodyRemovedHandler);
                scene->RegisterSceneSimulationFinishHandler(m_sceneSimulationFinishHandler);
                AZ_Printf("SimulationInterfaces", "Registered simulation body added handler\n");
                m_physicsScenesHandle = sceneHandle;
            });
        m_sceneSimulationFinishHandler = AzPhysics::SceneEvents::OnSceneSimulationStartHandler(
            [](AzPhysics::SceneHandle sceneHandle, float fixedDeltaTime)
            {
                AZ_UNUSED(sceneHandle);
                AZ_UNUSED(fixedDeltaTime);
//                for (const auto& bodyHandle : m_disabledBodies)
//                {
//                    auto* scene = GetSceneHelper(m_physicsScenesHandle);
//                    if (scene == nullptr)
//                    {
//                        return;
//                    }
//                    auto* body = scene->GetSimulatedBodyFromHandle(bodyHandle);
//                    if (body != nullptr)
//                    {
//                        AzPhysics::SimulatedBodyComponentRequestsBus::Event(body->GetEntityId(), &AzPhysics::SimulatedBodyComponentRequests::EnablePhysics);
//                    }
//                }
            });
        m_sceneRemovedHandler = AzPhysics::SystemEvents::OnSceneRemovedEvent::Handler(
            [this](AzPhysics::SceneHandle sceneHandle)
            {
                if (m_physicsScenesHandle == sceneHandle)
                {
                    m_entityIdToSimulatedEntityMap.clear();
                    m_simulatedEntityToEntityIdMap.clear();
                    m_simulationBodyAddedHandler.Disconnect();
                    m_simulationBodyRemovedHandler.Disconnect();
                    m_sceneSimulationFinishHandler.Disconnect();
                    m_physicsScenesHandle = AzPhysics::InvalidSceneHandle;
                }
            });
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "Physics system is not available.");
        physicsSystem->RegisterSceneAddedEvent(m_sceneAddedHandler);
        physicsSystem->RegisterSceneRemovedEvent(m_sceneRemovedHandler);

        SimulationInterfacesRequestBus::Handler::BusConnect();
    }

    void SimulationEntitiesManager::Deactivate()
    {
        if (m_sceneAddedHandler.IsConnected())
        {
            m_sceneAddedHandler.Disconnect();
        }
        if (m_simulationBodyAddedHandler.IsConnected())
        {
            m_simulationBodyAddedHandler.Disconnect();
        }

        if (m_sceneAddedHandler.IsConnected())
        {
            m_sceneAddedHandler.Disconnect();
        }
        if (m_simulationBodyRemovedHandler.IsConnected())
        {
            m_simulationBodyRemovedHandler.Disconnect();
        }
        m_physicsScenesHandle = AzPhysics::InvalidSceneHandle;

        SimulationInterfacesRequestBus::Handler::BusDisconnect();
    }


    AZStd::string SimulationEntitiesManager::AddSimulatedEntity(AZ::EntityId entityId)
    {
        if (!entityId.IsValid())
        {
            return "";
        }
        // check if entity is already registered
        auto findIt = m_entityIdToSimulatedEntityMap.find(entityId);
        if (findIt != m_entityIdToSimulatedEntityMap.end())
        {
            return findIt->second;
        }
        // Get O3DE entity name
        AZStd::string entityName = "Unknown";
        AZ::ComponentApplicationBus::BroadcastResult(entityName, &AZ::ComponentApplicationRequests::GetEntityName, entityId);
        // Generate unique simulated entity name
        AZStd::string simulatedEntityName = entityName;
        // check if name is unique
        auto otherEntityIt = m_simulatedEntityToEntityIdMap.find(simulatedEntityName);
        if (otherEntityIt != m_simulatedEntityToEntityIdMap.end())
        {
            // name is not unique, add entityId to name
            simulatedEntityName = AZStd::string::format("%s_%s", entityName.c_str(), entityId.ToString().c_str());
        }
        // register entity
        m_simulatedEntityToEntityIdMap[simulatedEntityName] = entityId;
        m_entityIdToSimulatedEntityMap[entityId] = simulatedEntityName;
        return simulatedEntityName;
    }

    void SimulationEntitiesManager::RemoveSimulatedEntity(AZ::EntityId entityId)
    {
        auto findIt = m_entityIdToSimulatedEntityMap.find(entityId);
        if (findIt != m_entityIdToSimulatedEntityMap.end())
        {
            const auto& simulatedEntityName = findIt->second;
            m_entityIdToSimulatedEntityMap.erase(findIt);
            m_simulatedEntityToEntityIdMap.erase(simulatedEntityName);
        }
    }

    AZStd::vector<AZStd::string> SimulationEntitiesManager::GetEntities(const EntityFilter& filter)
    {
        const bool reFilter = !filter.m_filter.empty();
        const bool shapeCastFilter = filter.m_bounds_shape != nullptr;

        AZStd::vector<AZStd::string> entities;
        if (!shapeCastFilter)
        {
            // get all entities from the map
            entities.reserve(m_entityIdToSimulatedEntityMap.size());
            AZStd::transform(
                m_entityIdToSimulatedEntityMap.begin(),
                m_entityIdToSimulatedEntityMap.end(),
                AZStd::back_inserter(entities),
                [](const auto& pair)
                {
                    return pair.second;
                });
        }
        else
        {
            auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
            AZ_Assert(sceneInterface, "Physics scene interface is not available.");

            if (m_physicsScenesHandle == AzPhysics::InvalidSceneHandle)
            {
                return entities;
            }

            AzPhysics::OverlapRequest request;
            request.m_shapeConfiguration = filter.m_bounds_shape;
            request.m_pose = filter.m_bounds_pose;
            request.m_maxResults = AZStd::numeric_limits<AZ::u32>::max();

            AzPhysics::SceneQueryHits result = sceneInterface->QueryScene(m_physicsScenesHandle, &request);
            for (const auto& hit : result.m_hits)
            {
                const AZ::EntityId entityId = hit.m_entityId;
                auto findIt = m_entityIdToSimulatedEntityMap.find(entityId);
                if (findIt != m_entityIdToSimulatedEntityMap.end())
                {
                    entities.push_back(findIt->second);
                }
            }
        }
        if (reFilter)
        {
            const AZStd::vector<AZStd::string> prefilteredEntities = AZStd::move(entities);
            entities.clear();
            const AZStd::regex regex(filter.m_filter);
            if (regex.Valid())
            {
                AZStd::ranges::copy_if(
                    prefilteredEntities.begin(),
                    prefilteredEntities.end(),
                    AZStd::back_inserter(entities),
                    [&regex](const AZStd::string& entityName)
                    {
                        return AZStd::regex_search(entityName, regex);
                    });
            }
        }
        return entities;
    }

    EntityState SimulationEntitiesManager::GetEntityState(const AZStd::string& name)
    {
        const auto findIt = m_simulatedEntityToEntityIdMap.find(name);
        AZ_Error("SimulationInterfaces",findIt != m_simulatedEntityToEntityIdMap.end(), "Entity %s not found", name.c_str());
        if (findIt != m_simulatedEntityToEntityIdMap.end())
        {
            EntityState entityState {};
            const AZ::EntityId entityId = findIt->second;
            AZ_Assert(entityId.IsValid(), "EntityId is not valid");
            AZ::TransformBus::EventResult(entityState.m_pose, entityId, &AZ::TransformBus::Events::GetWorldTM);

            AZ::Vector3 linearVelocity = AZ::Vector3::CreateZero();
            Physics::RigidBodyRequestBus::EventResult(linearVelocity, entityId, &Physics::RigidBodyRequests::GetLinearVelocity);

            AZ::Vector3 angularVelocity = AZ::Vector3::CreateZero();
            Physics::RigidBodyRequestBus::EventResult(angularVelocity, entityId, &Physics::RigidBodyRequests::GetAngularVelocity);

            // transform linear and angular velocities to entity frame
            AZ::Transform entityTransformInv = entityState.m_pose.GetInverse();
            entityState.m_twist_linear = entityTransformInv.TransformVector(linearVelocity);
            entityState.m_twist_angular = entityTransformInv.TransformVector(angularVelocity);
            return entityState;
        }
        return {};
    }

    bool SimulationEntitiesManager::SetEntityState(const AZStd::string& name, const EntityState& state)
    {
        const auto findIt = m_simulatedEntityToEntityIdMap.find(name);
        if (findIt != m_simulatedEntityToEntityIdMap.end())
        {
            const AZ::EntityId entityId = findIt->second;
            AZ_Assert(entityId.IsValid(), "EntityId is not valid");

            // get entity and all descendants
            AZStd::vector<AZ::EntityId> entityAndDescendants;
            AZ::TransformBus::EventResult(entityAndDescendants, entityId, &AZ::TransformBus::Events::GetEntityAndAllDescendants);

            if (state.m_pose.IsOrthogonal())
            {
                // disable simulation for all entities
                AZStd::map<AZ::EntityId, AZ::Transform> entityTransforms;
                for (const auto& descendant : entityAndDescendants)
                {
                    // get name
                    AZStd::string entityName = "Unknown";
                    AZ::ComponentApplicationBus::BroadcastResult(entityName, &AZ::ComponentApplicationRequests::GetEntityName, descendant);
                    AZ_Printf("SimulationInterfaces", "Disable physics for entity %s\n",entityName.c_str());
                   Physics::RigidBodyRequestBus::Event(descendant, &Physics::RigidBodyRequests::DisablePhysics);
                }

                AZ::TransformBus::Event(entityId, &AZ::TransformBus::Events::SetLocalTM, state.m_pose);

                for (const auto& descendant : entityAndDescendants)
                {
                   Physics::RigidBodyRequestBus::Event(descendant, &Physics::RigidBodyRequests::EnablePhysics);
                   Physics::RigidBodyRequestBus::Event(descendant, &Physics::RigidBodyRequests::SetAngularVelocity, AZ::Vector3::CreateZero());
                   Physics::RigidBodyRequestBus::Event(descendant, &Physics::RigidBodyRequests::SetLinearVelocity, AZ::Vector3::CreateZero());
                }
            }
            if (!state.m_twist_linear.IsClose(AZ::Vector3::CreateZero(), AZ::Constants::FloatEpsilon) ||
                !state.m_twist_angular.IsClose(AZ::Vector3::CreateZero(), AZ::Constants::FloatEpsilon))
            {
                // get rigid body
                AzPhysics::RigidBody* rigidBody = nullptr;
                Physics::RigidBodyRequestBus::EventResult(rigidBody, entityId, &Physics::RigidBodyRequests::GetRigidBody);
                if (rigidBody != nullptr)
                {
                    SetRigidBodyVelocities(rigidBody, state);
                }
            }

        }
        return false;
    }

    AZStd::unordered_map<AZStd::string, EntityState> SimulationEntitiesManager::GetEntitiesStates(const EntityFilter& filter)
    {
        AZStd::unordered_map<AZStd::string, EntityState> entitiesStates;
        const auto entities = GetEntities(filter);
        for (const auto& entity : entities)
        {
            entitiesStates[entity] = GetEntityState(entity);
        }
        return entitiesStates;
    }

} // namespace SimulationInterfaces
