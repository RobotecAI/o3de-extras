/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

//#include "Components/EditorWhiteBoxColliderComponent.h"
//#include "Components/WhiteBoxColliderComponent.h"
//#include "WhiteBox/EditorWhiteBoxComponentBus.h"
//#include "WhiteBox/WhiteBoxToolApi.h"
//#include "WhiteBoxTestFixtures.h"
//#include "WhiteBoxTestUtil.h"

#include <AzCore/UserSettings/UserSettingsComponent.h>
#include <AzToolsFramework/UnitTest/ToolsTestApplication.h>

#include <QApplication>

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Slice/SliceAssetHandler.h>
#include <AzQtComponents/Utilities/QtPluginPaths.h>
#include <AzTest/GemTestEnvironment.h>
#include <AzToolsFramework/Commands/PreemptiveUndoCache.h>
#include <AzToolsFramework/Entity/EditorEntityContextComponent.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>

#include "RobotImporter/URDF/InertialsMaker.h"
#include "RobotImporter/URDF/CollidersMaker.h"


namespace UnitTest
{
   class URDFImporterPhysicsTestEnvironment : public AZ::Test::GemTestEnvironment
   {
       // AZ::Test::GemTestEnvironment overrides ...
       void AddGemsAndComponents() override;
       AZ::ComponentApplication* CreateApplicationInstance() override;
       void PostSystemEntityActivate() override;

   public:
       URDFImporterPhysicsTestEnvironment() = default;
       ~URDFImporterPhysicsTestEnvironment() override = default;
   };

   void URDFImporterPhysicsTestEnvironment::AddGemsAndComponents()
   {
       AddDynamicModulePaths({"PhysX.Editor.Gem"});
       AddComponentDescriptors(
           {});
   }

   AZ::ComponentApplication* URDFImporterPhysicsTestEnvironment::CreateApplicationInstance()
   {
       // Using ToolsTestApplication to have AzFramework and AzToolsFramework components.
       return aznew UnitTest::ToolsTestApplication("UrdfImpoterPhysics");
   }

   void URDFImporterPhysicsTestEnvironment::PostSystemEntityActivate()
   {
       AZ::UserSettingsComponentRequestBus::Broadcast(&AZ::UserSettingsComponentRequests::DisableSaveOnFinalize);
   }

   class UrdfImportPhysicsFixture : public ::testing::Test
   {
   };

   TEST_F(UrdfImportPhysicsFixture, URDFImporterIntertiaMaker)
   {

       AZ::Entity entity;
       entity.CreateComponent<AzToolsFramework::Components::TransformComponent>();
       entity.Init();

       urdf::InertialSharedPtr inertial = std::make_shared<urdf::Inertial>();

       ROS2::InertialsMaker inertialsMaker;
       inertialsMaker.AddInertial(inertial, entity.GetId());
       entity.Activate();

       EXPECT_EQ(entity.GetState(), AZ::Entity::State::Active);

   }

   TEST_F(UrdfImportPhysicsFixture, URDFImporterCollisionSphere)
   {

       AZ::Entity entity;
       entity.CreateComponent<AzToolsFramework::Components::TransformComponent>();
       entity.Init();

       urdf::CollisionSharedPtr collision = std::make_shared<urdf::Collision>();
       urdf::SphereSharedPtr sphere = std::make_shared<urdf::Sphere>();
       sphere->radius = 2.0f;
       collision->geometry = sphere;

       urdf::LinkSharedPtr link = std::make_shared<urdf::Link>();
       link->collision = collision;
       link->inertial = std::make_shared<urdf::Inertial>();

       AZStd::shared_ptr<ROS2::Utils::UrdfAssetMap> mappings = AZStd::make_shared<ROS2::Utils::UrdfAssetMap>();
       ROS2::CollidersMaker collidersMaker (mappings);
       ROS2::InertialsMaker inertialsMaker;

       inertialsMaker.AddInertial(link->inertial, entity.GetId());
       collidersMaker.AddColliders(link, entity.GetId());

       entity.Activate();

       EXPECT_EQ(entity.GetState(), AZ::Entity::State::Active);
   }


} // namespace UnitTest

// required to support running integration tests with Qt and PhysX
AZTEST_EXPORT int AZ_UNIT_TEST_HOOK_NAME(int argc, char** argv)
{
   ::testing::InitGoogleMock(&argc, argv);
   AzQtComponents::PrepareQtPaths();
   QApplication app(argc, argv);
   AZ::Test::printUnusedParametersWarning(argc, argv);
   AZ::Test::addTestEnvironments({new UnitTest::URDFImporterPhysicsTestEnvironment()});
   int result = RUN_ALL_TESTS();
   return result;
}

IMPLEMENT_TEST_EXECUTABLE_MAIN();
