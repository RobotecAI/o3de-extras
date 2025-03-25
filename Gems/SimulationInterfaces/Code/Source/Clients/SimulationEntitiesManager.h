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
#include <SimulationInterfaces/SimulationInterfacesBus.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Physics/PhysicsScene.h>
namespace SimulationInterfaces
{
    class SimulationEntitiesManager
        : public AZ::Component
        , protected SimulationInterfacesRequestBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(SimulationEntitiesManager);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);


        SimulationEntitiesManager();
        ~SimulationEntitiesManager();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // SimulationInterfacesRequestBus interface implementation
        AZStd::vector<AZStd::string> GetEntities(const EntityFilter& filter) override;
        EntityState GetEntityState(const AZStd::string& name) override;
        AZStd::unordered_map<AZStd::string, EntityState> GetEntitiesStates(const EntityFilter& filter) override;
        bool SetEntityState(const AZStd::string& name, const EntityState& state) override;
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////


        //! Registers simulated entity to entity id mapping.
        //! Note that the entityId will be registered under unique name.
        //! \param entityId The entity id to register
        //! \return returns the simulated entity name
        AZStd::string AddSimulatedEntity(AZ::EntityId entityId);

        //! Removes simulated entity from the mapping.
        void RemoveSimulatedEntity(AZ::EntityId entityId);


    private:
        AzPhysics::SceneEvents::OnSimulationBodyAdded::Handler m_simulationBodyAddedHandler;
        AzPhysics::SceneEvents::OnSimulationBodyRemoved::Handler m_simulationBodyRemovedHandler;
        AzPhysics::SceneEvents::OnSceneSimulationFinishHandler m_sceneSimulationFinishHandler;

        AzPhysics::SystemEvents::OnSceneAddedEvent::Handler m_sceneAddedHandler;
        AzPhysics::SystemEvents::OnSceneRemovedEvent::Handler m_sceneRemovedHandler;
        AzPhysics::SceneHandle m_physicsScenesHandle = AzPhysics::InvalidSceneHandle;
        AZStd::unordered_map<AZStd::string, AZ::EntityId> m_simulatedEntityToEntityIdMap;
        AZStd::unordered_map<AZ::EntityId, AZStd::string> m_entityIdToSimulatedEntityMap;
        AZStd::unordered_set<AzPhysics::SimulatedBodyHandle> m_disabledBodies;

    };

} // namespace SimulationInterfaces
