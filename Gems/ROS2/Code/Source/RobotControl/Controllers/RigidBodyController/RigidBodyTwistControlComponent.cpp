/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RigidBodyTwistControlComponent.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>

namespace ROS2
{
    void RigidBodyTwistControlComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<RigidBodyTwistControlComponent, AZ::Component>()
                ->Version(1)
                ->Field("DisableLinearVelocityXOverriding", &RigidBodyTwistControlComponent::m_disableLinearVelocityXOverriding)
                ->Field("DisableLinearVelocityYOverriding", &RigidBodyTwistControlComponent::m_disableLinearVelocityYOverriding)
                ->Field("DisableLinearVelocityZOverriding", &RigidBodyTwistControlComponent::m_disableLinearVelocityZOverriding)
                ->Field("DisableAngularVelocityXOverriding", &RigidBodyTwistControlComponent::m_disableAngularVelocityXOverriding)
                ->Field("DisableAngularVelocityYOverriding", &RigidBodyTwistControlComponent::m_disableAngularVelocityYOverriding)
                ->Field("DisableAngularVelocityZOverriding", &RigidBodyTwistControlComponent::m_disableAngularVelocityZOverriding);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<RigidBodyTwistControlComponent>("Rigid Body Twist Control", "Simple control through RigidBody")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/RigidBodyTwistControl.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/RigidBodyTwistControl.svg")
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Linear Axis Overrides")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, false)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RigidBodyTwistControlComponent::m_disableLinearVelocityXOverriding,
                        "Disable linear velocity X axis override",
                        "Disable overriding X axis")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RigidBodyTwistControlComponent::m_disableLinearVelocityYOverriding,
                        "Disable linear velocity Y axis override",
                        "Disable overriding Y axis")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RigidBodyTwistControlComponent::m_disableLinearVelocityZOverriding,
                        "Disable linear velocity Z axis override",
                        "Disable overriding Z axis")
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Angular Axis Overrides")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, false)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RigidBodyTwistControlComponent::m_disableAngularVelocityXOverriding,
                        "Disable angular velocity X axis override",
                        "Disable overriding X axis")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RigidBodyTwistControlComponent::m_disableAngularVelocityYOverriding,
                        "Disable angular velocity Y axis override",
                        "Disable overriding Y axis")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RigidBodyTwistControlComponent::m_disableAngularVelocityZOverriding,
                        "Disable angular velocity Z axis override",
                        "Disable overriding Z axis");
            }
        }
    }

    void RigidBodyTwistControlComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        TwistNotificationBus::Handler::BusConnect(GetEntityId());
    }

    void RigidBodyTwistControlComponent::Deactivate()
    {
        TwistNotificationBus::Handler::BusDisconnect();
        if (m_sceneFinishSimHandler.IsConnected())
        {
            m_sceneFinishSimHandler.Disconnect();
        }
        if (AZ::TickBus::Handler::BusIsConnected())
        {
            AZ::TickBus::Handler::BusDisconnect();
        }
    }

    void RigidBodyTwistControlComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "No scene interface");
        if (!sceneInterface)
        {
            return;
        }
        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene handle");

        AzPhysics::RigidBody* rigidBody = nullptr;
        Physics::RigidBodyRequestBus::EventResult(rigidBody, GetEntityId(), &Physics::RigidBodyRequests::GetRigidBody);
        AZ_Warning("RigidBodyTwistControlComponent", rigidBody, "No rigid body found for entity %s", GetEntity()->GetName().c_str());
        if (!rigidBody)
        {
            return;
        }
        m_bodyHandle = rigidBody->m_bodyHandle;
        m_sceneFinishSimHandler = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
            [this, sceneInterface]([[maybe_unused]] AzPhysics::SceneHandle sceneHandle, float fixedDeltaTime)
            {
                auto* rigidBody = sceneInterface->GetSimulatedBodyFromHandle(sceneHandle, m_bodyHandle);
                AZ_Assert(sceneInterface, "No body found for previously given handle");

                // Convert local steering to world frame
                const AZ::Transform robotTransform = rigidBody->GetTransform();
                auto linearVelocityGlobal = robotTransform.TransformVector(m_linearVelocityLocal);
                auto angularVelocityGlobal = robotTransform.TransformVector(m_angularVelocityLocal);
                AZ::Vector3 linearVelocity;
                AZ::Vector3 angularVelocity;
                Physics::RigidBodyRequestBus::EventResult(linearVelocity, GetEntityId(), &Physics::RigidBodyRequests::GetLinearVelocity);
                Physics::RigidBodyRequestBus::EventResult(angularVelocity, GetEntityId(), &Physics::RigidBodyRequests::GetAngularVelocity);
                if (m_disableLinearVelocityXOverriding)
                {
                    linearVelocityGlobal.SetX(linearVelocity.GetX());
                }
                if (m_disableLinearVelocityYOverriding)
                {
                    linearVelocityGlobal.SetY(linearVelocity.GetY());
                }
                if (m_disableLinearVelocityZOverriding)
                {
                    linearVelocityGlobal.SetZ(linearVelocity.GetZ());
                }
                if (m_disableAngularVelocityXOverriding)
                {
                    angularVelocityGlobal.SetX(angularVelocity.GetX());
                }
                if (m_disableAngularVelocityYOverriding)
                {
                    angularVelocityGlobal.SetY(angularVelocity.GetY());
                }
                if (m_disableAngularVelocityZOverriding)
                {
                    angularVelocityGlobal.SetZ(angularVelocity.GetZ());
                }

                Physics::RigidBodyRequestBus::Event(GetEntityId(), &Physics::RigidBodyRequests::SetLinearVelocity, linearVelocityGlobal);
                Physics::RigidBodyRequestBus::Event(GetEntityId(), &Physics::RigidBodyRequests::SetAngularVelocity, angularVelocityGlobal);
            },
            aznumeric_cast<int32_t>(AzPhysics::SceneEvents::PhysicsStartFinishSimulationPriority::Components));
        sceneInterface->RegisterSceneSimulationFinishHandler(defaultSceneHandle, m_sceneFinishSimHandler);
        AZ::TickBus::Handler::BusDisconnect();
    }

    void RigidBodyTwistControlComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2RobotControl"));
        required.push_back(AZ_CRC_CE("PhysicsRigidBodyService"));
    }

    void RigidBodyTwistControlComponent::TwistReceived(const AZ::Vector3& linear, const AZ::Vector3& angular)
    {
        m_linearVelocityLocal = linear;
        m_angularVelocityLocal = angular;
    }
} // namespace ROS2
