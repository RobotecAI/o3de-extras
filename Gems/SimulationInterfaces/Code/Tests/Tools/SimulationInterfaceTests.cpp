
/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Asset/AssetManagerComponent.h>
#include <AzCore/Component/ComponentApplication.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Slice/SliceAssetHandler.h>
#include <AzCore/UserSettings/UserSettingsComponent.h>
#include <AzCore/std/containers/array.h>
#include <AzCore/std/string/string_view.h>
#include <AzQtComponents/Utilities/QtPluginPaths.h>
#include <AzTest/GemTestEnvironment.h>
#include <AzToolsFramework/Entity/EditorEntityContextComponent.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>
#include <AzToolsFramework/UnitTest/AzToolsFrameworkTestHelpers.h>
#include <AzToolsFramework/UnitTest/ToolsTestApplication.h>

#include <QApplication>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <SimulationInterfaces/SimulationInterfacesBus.h>
#include "Clients/SimulationInterfacesSystemComponent.h"
#include <AzFramework/Physics/SystemBus.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/Configuration/SystemConfiguration.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>

namespace UnitTest
{
    class SimulationInterfaceTestEnvironment : public AZ::Test::GemTestEnvironment
    {
        // AZ::Test::GemTestEnvironment overrides ...
        void AddGemsAndComponents() override;
        AZ::ComponentApplication* CreateApplicationInstance() override;
        void PostSystemEntityActivate() override;

    public:
        SimulationInterfaceTestEnvironment() = default;
        ~SimulationInterfaceTestEnvironment() override = default;
    };

    void SimulationInterfaceTestEnvironment::AddGemsAndComponents()
    {
        constexpr AZStd::array<AZStd::string_view,3> requiredGems =
        {
            "PhysX5", // required for PhysX Dynamic
            "LmbrCentral", // for shapes
            "SimulationInterfaces"
        };
        AddActiveGems(requiredGems);
        AddDynamicModulePaths({"PhysX5.Gem"});
        AddDynamicModulePaths({"LmbrCentral"});
        AddComponentDescriptors({
            SimulationInterfaces::SimulationInterfacesSystemComponent::CreateDescriptor(),
        });
        AddRequiredComponents({
            SimulationInterfaces::SimulationInterfacesSystemComponent::TYPEINFO_Uuid()
        });
    }


    void SimulationInterfaceTestEnvironment::PostSystemEntityActivate()
    {
        AZ::UserSettingsComponentRequestBus::Broadcast(&AZ::UserSettingsComponentRequests::DisableSaveOnFinalize);
    }


    AZ::ComponentApplication* SimulationInterfaceTestEnvironment::CreateApplicationInstance()
    {
        // Using ToolsTestApplication to have AzFramework and AzToolsFramework components.
        return aznew UnitTest::ToolsTestApplication("SimulationInterfaceTestEnvironment");
    }


    class SimulationInterfaceTestFixture : public ::testing::Test
        , protected Physics::DefaultWorldBus::Handler
    {
    protected:
        constexpr static auto PhysXRigidBodyComponentTypeId = "{D4E52A70-BDE1-4819-BD3C-93AB3F4F3BE3}"; // From PhysX
        constexpr static auto PhysXStaticBodyComponentTypeId = "{A2CCCD3D-FB31-4D65-8DCD-2CD7E1D09538}"; // From PhysX
        constexpr static auto PhysXShapeColliderComponentTypeId = "{30CC9E77-378C-49DF-9617-6BF191901FE0}"; // From PhysX
        constexpr static auto PhysXSphereColliderComponentTypeId = "{108CD341-E5C3-4AE1-B712-21E81ED6C277}"; // From PhysX
        constexpr static auto SphereShapeComponentTypeId = "{E24CBFF0-2531-4F8D-A8AB-47AF4D54BCD2}"; // From LmbrCentral
        
        void SetUp() override;
        void TearDown() override;

        AZ::EntityId CreateEntityWithStaticBodyComponent(const AZStd::string& entityName, const AZ::Transform& transform);

        void DeleteEntity(const AZ::EntityId& entityId);
        void ClearEntities();
        // DefaultWorldBus
        AzPhysics::SceneHandle GetDefaultSceneHandle() const override;

        AzPhysics::Scene* m_defaultScene = nullptr;
        AzPhysics::SceneHandle m_testSceneHandle = AzPhysics::InvalidSceneHandle;

        AZStd::unordered_map<AZ::EntityId, AZStd::unique_ptr<AZ::Entity>> m_entities;

    };

    AzPhysics::SceneHandle SimulationInterfaceTestFixture::GetDefaultSceneHandle() const
    {
        return m_testSceneHandle;
    }

    AZ::EntityId SimulationInterfaceTestFixture::CreateEntityWithStaticBodyComponent(const AZStd::string& entityName, const AZ::Transform& transform)
    {
        AZStd::unique_ptr<AZ::Entity> entity = AZStd::make_unique<AZ::Entity>(entityName.c_str());
        auto * transformComponent = entity->CreateComponent(AZ::TransformComponentTypeId);
        AZ_Assert(transformComponent, "Failed to create TransformComponent");
        auto * transformInterface = azrtti_cast<AZ::TransformInterface*>(transformComponent);
        AZ_Assert(transformInterface, "Failed to get TransformInterface");
        transformInterface->SetWorldTM(transform);
        entity->CreateComponent(AZ::Uuid(PhysXRigidBodyComponentTypeId));
        entity->CreateComponent(AZ::Uuid(PhysXShapeColliderComponentTypeId));
        entity->CreateComponent(AZ::Uuid(SphereShapeComponentTypeId));
        entity->Init();
        entity->Activate();
        AZ_Assert(entity->GetState()==AZ::Entity::State::Active, "Entity is not active");

        auto id = entity->GetId();
        m_entities.emplace(AZStd::make_pair(id, AZStd::move(entity)));

        return id;
    }

    void SimulationInterfaceTestFixture::ClearEntities()
    {
        for (auto& entity : m_entities)
        {
            entity.second->Deactivate();
        }
        m_entities.clear();
    }
    void SimulationInterfaceTestFixture::DeleteEntity(const AZ::EntityId& entityId)
    {
        auto findIt = m_entities.find(entityId);
        if (findIt != m_entities.end())
        {
            findIt->second->Deactivate();
            m_entities.erase(findIt);
        }
    }

    void SimulationInterfaceTestFixture::SetUp()
    {

        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            AzPhysics::SceneConfiguration sceneConfiguration = physicsSystem->GetDefaultSceneConfiguration();
            sceneConfiguration.m_sceneName = AzPhysics::DefaultPhysicsSceneName;
            m_testSceneHandle = physicsSystem->AddScene(sceneConfiguration);
            m_defaultScene = physicsSystem->GetScene(m_testSceneHandle);
        }

        Physics::DefaultWorldBus::Handler::BusConnect();

    }

    void SimulationInterfaceTestFixture::TearDown()
    {
        Physics::DefaultWorldBus::Handler::BusDisconnect();
        m_defaultScene = nullptr;

        //Clean up the Test scene
        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            physicsSystem->RemoveScene(m_testSceneHandle);
        }
        m_testSceneHandle = AzPhysics::InvalidSceneHandle;
    }


    TEST_F(SimulationInterfaceTestFixture, EmptyScene)
    {
        using namespace SimulationInterfaces;
        AZStd::vector<AZStd::string> entities;
        SimulationInterfacesRequestBus::BroadcastResult(entities, &SimulationInterfacesRequestBus::Events::GetEntities, EntityFilter());
        EXPECT_EQ(entities.size(),0);
    }

    TEST_F(SimulationInterfaceTestFixture, AddSimulatedEntityThenRemove)
    {
        using namespace SimulationInterfaces;
        const AZ::EntityId entityId1 = CreateEntityWithStaticBodyComponent("Foo", AZ::Transform::CreateIdentity());
        const AZ::EntityId entityId2 = CreateEntityWithStaticBodyComponent("Bar", AZ::Transform::CreateIdentity());

        AZStd::vector<AZStd::string> entities;
        SimulationInterfacesRequestBus::BroadcastResult(entities, &SimulationInterfacesRequestBus::Events::GetEntities, EntityFilter());
        AZ_Assert(entities.size() == 2, "Number of simulation entities: %d", entities.size());
        DeleteEntity(entityId1);

        AZStd::vector<AZStd::string> entities2;
        SimulationInterfacesRequestBus::BroadcastResult(entities2, &SimulationInterfacesRequestBus::Events::GetEntities, EntityFilter());
        EXPECT_EQ(entities2.size(),1);

        DeleteEntity(entityId2);
        AZStd::vector<AZStd::string> entities3;
        SimulationInterfacesRequestBus::BroadcastResult(entities3, &SimulationInterfacesRequestBus::Events::GetEntities, EntityFilter());
        EXPECT_EQ(entities3.size(),0);
    }

    TEST_F(SimulationInterfaceTestFixture, AddEntitiesWithDupName)
    {
        using namespace SimulationInterfaces;

        const AZ::EntityId entityId1 = CreateEntityWithStaticBodyComponent("Bar1", AZ::Transform::CreateIdentity());
        const AZ::EntityId entityId2 = CreateEntityWithStaticBodyComponent("Bar1", AZ::Transform::CreateIdentity());
        AZStd::vector<AZStd::string> entities;

        SimulationInterfacesRequestBus::BroadcastResult(entities, &SimulationInterfacesRequestBus::Events::GetEntities, EntityFilter());
        EXPECT_EQ(entities.size(),2);
        EXPECT_NE(entities[0],entities[1]);
        DeleteEntity(entityId1);
        DeleteEntity(entityId2);
    }

    TEST_F(SimulationInterfaceTestFixture, TestShapeFilter)
    {
        // This test is disabled since due to some issue outside to this gem, the rigid body is created without the collider shape
        // and the filter is not applied. This test will be enabled once the issue is resolved.
        return;
        using namespace SimulationInterfaces;
        const AZ::EntityId entityId1 = CreateEntityWithStaticBodyComponent("Inside", AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 0.0f)));
        const AZ::EntityId entityId2 = CreateEntityWithStaticBodyComponent("Outside", AZ::Transform::CreateTranslation(AZ::Vector3(10.0f, 0.0f, 0.0f)));


        EntityFilter filter;
        filter.m_bounds = AZStd::make_shared<Physics::SphereShapeConfiguration>(2.0f);

        AZStd::vector<AZStd::string> entities;
        SimulationInterfacesRequestBus::BroadcastResult(entities, &SimulationInterfacesRequestBus::Events::GetEntities, filter);
        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        physicsSystem->Simulate(1.0f / 60.0f);

        EXPECT_EQ(entities.size(),1);
        if (entities.size() > 0)
        {
            EXPECT_EQ(entities.front(), "Inside");
        }
        DeleteEntity(entityId1);
        DeleteEntity(entityId2);
    }


} // namespace UnitTest

// required to support running integration tests with Qt and PhysX
AZTEST_EXPORT int AZ_UNIT_TEST_HOOK_NAME(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    AzQtComponents::PrepareQtPaths();
    QApplication app(argc, argv);
    AZ::Test::printUnusedParametersWarning(argc, argv);
    AZ::Test::addTestEnvironments({ new UnitTest::SimulationInterfaceTestEnvironment() });
    int result = RUN_ALL_TESTS();
    return result;
}

IMPLEMENT_TEST_EXECUTABLE_MAIN();
