/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationInterfacesSystemComponent.h"

#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Console/IConsole.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
namespace SimulationInterfacesCommands
{
    using namespace SimulationInterfaces;
    static void simulationinterfaces_GetEntitiesNoFilter(const AZ::ConsoleCommandContainer& arguments)
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
        if (arguments.size() > 0)
        {
            AZStd::string argument{ arguments[0] };
            sphereShape = AZStd::stof(argument);
        }
        // Put your command handler code here
        AZ_Printf("SimulationInterfaces", "simulationinterfaces_GetEntities in radius %f \n", sphereShape);
        EntityFilter filter;
        filter.m_bounds = AZStd::make_shared<Physics::SphereShapeConfiguration>(sphereShape);

        AZStd::vector<AZStd::string> entities;
        SimulationInterfacesRequestBus::BroadcastResult(entities, &SimulationInterfacesRequestBus::Events::GetEntities, filter);
        AZ_Printf("SimulationInterfaces", "Number of simulation entities: %d\n", entities.size());
        for (const auto& entity : entities)
        {
            AZ_Printf("SimulationInterfaces", "      - %s\n", entity.c_str());
        }
    }

    AZ_CONSOLEFREEFUNC(
        simulationinterfaces_GetEntitiesNoFilter, AZ::ConsoleFunctorFlags::DontReplicate, "Get all simulated entities in the scene.");
    AZ_CONSOLEFREEFUNC(
        simulationinterfaces_GetEntitiesSphere, AZ::ConsoleFunctorFlags::DontReplicate, "Get all simulated entities in the radius.");

} // namespace SimulationInterfacesCommands
namespace SimulationInterfaces
{

    AZ_COMPONENT_IMPL(
        SimulationInterfacesSystemComponent, "SimulationInterfacesSystemComponent", SimulationInterfacesSystemComponentTypeId);

    void SimulationInterfacesSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SimulationInterfacesSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void SimulationInterfacesSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("SimulationInterfacesService"));
    }

    void SimulationInterfacesSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("SimulationInterfacesService"));
    }

    void SimulationInterfacesSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("PhysicsService"));
    }

    void SimulationInterfacesSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    SimulationInterfacesSystemComponent::SimulationInterfacesSystemComponent()
    {
        if (SimulationInterfacesInterface::Get() == nullptr)
        {
            SimulationInterfacesInterface::Register(this);
        }
    }

    SimulationInterfacesSystemComponent::~SimulationInterfacesSystemComponent()
    {
        if (SimulationInterfacesInterface::Get() == this)
        {
            SimulationInterfacesInterface::Unregister(this);
        }
    }

    void SimulationInterfacesSystemComponent::Init()
    {
    }

    AzPhysics::Scene* GetSceneHelper(AzPhysics::SceneHandle sceneHandle)
    {
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "Physics system is not available.");
        AzPhysics::Scene* scene = physicsSystem->GetScene(sceneHandle);
        return scene;
    }
    void SimulationInterfacesSystemComponent::Activate()
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
                    AZ_Warning("SimulationInterfaces", shapeCount > 0, "Entity %s has no shapes, it won't be available by bound search", rigidBody->GetEntityId().ToString().c_str());
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
                AZ_Printf("SimulationInterfaces", "Registered simulation body added handler\n");
                m_physicsScenesHandle = sceneHandle;
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
                    m_physicsScenesHandle = AzPhysics::InvalidSceneHandle;
                }
            });
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "Physics system is not available.");
        physicsSystem->RegisterSceneAddedEvent(m_sceneAddedHandler);
        physicsSystem->RegisterSceneRemovedEvent(m_sceneRemovedHandler);
        SimulationInterfacesRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void SimulationInterfacesSystemComponent::Deactivate()
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

        AZ::TickBus::Handler::BusDisconnect();
        SimulationInterfacesRequestBus::Handler::BusDisconnect();
    }

    void SimulationInterfacesSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    AZStd::string SimulationInterfacesSystemComponent::AddSimulatedEntity(AZ::EntityId entityId)
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

    void SimulationInterfacesSystemComponent::RemoveSimulatedEntity(AZ::EntityId entityId)
    {
        auto findIt = m_entityIdToSimulatedEntityMap.find(entityId);
        if (findIt != m_entityIdToSimulatedEntityMap.end())
        {
            const auto& simulatedEntityName = findIt->second;
            m_entityIdToSimulatedEntityMap.erase(findIt);
            m_simulatedEntityToEntityIdMap.erase(simulatedEntityName);
        }
    }

    AZStd::vector<AZStd::string> SimulationInterfacesSystemComponent::GetEntities(const EntityFilter& filter)
    {
        //        const bool posixFilter = !filter.m_filter.empty();
        const bool shapeCastFilter = filter.m_bounds != nullptr;

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
            request.m_shapeConfiguration = filter.m_bounds;
            request.m_pose = AZ::Transform::CreateIdentity();
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
        return entities;
    }

} // namespace SimulationInterfaces
