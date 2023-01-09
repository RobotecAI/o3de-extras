/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include "SkidSteeringControlComponent.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <VehicleDynamics/WheelControllerComponent.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>
namespace ROS2
{
   void SkidSteeringControlComponent::Reflect(AZ::ReflectContext* context)
   {
       if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
       {
           serialize->Class<SkidSteeringControlComponent, AZ::Component>()->Version(1)
               ->Field("Wheels", &SkidSteeringControlComponent::m_wheels)
               ->Field("WheelRadius", &SkidSteeringControlComponent::m_wheelRadius)
               ->Field("Limits", &SkidSteeringControlComponent::m_limits);
           if (AZ::EditContext* ec = serialize->GetEditContext())
           {
               ec->Class<SkidSteeringControlComponent>("Skid steering Twist Control", "")
                   ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                   ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                   ->DataElement(AZ::Edit::UIHandlers::Default, &SkidSteeringControlComponent::m_wheels, "Wheels", "List of wheels")
                   ->DataElement(AZ::Edit::UIHandlers::Default, &SkidSteeringControlComponent::m_wheelRadius, "Wheel Radius", "")
                   ->DataElement(AZ::Edit::UIHandlers::Default, &SkidSteeringControlComponent::m_limits, "Limits", "")
                   ->Attribute(AZ::Edit::Attributes::Category, "ROS2");
           }
       }
   }

   void SkidSteeringControlComponent::Activate()
   {
       TwistNotificationBus::Handler::BusConnect(GetEntityId());
       m_wheelsData.clear();
   }

   void SkidSteeringControlComponent::Deactivate()
   {
       TwistNotificationBus::Handler::BusDisconnect();
   }

   void SkidSteeringControlComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
   {
       required.push_back(AZ_CRC_CE("ROS2RobotControl"));
   }

   void SkidSteeringControlComponent::TwistReceived(const AZ::Vector3& linear, const AZ::Vector3& angular)
   {

       if (m_wheelsData.empty())
       {
           for (auto id : m_wheels)
           {
               auto hinge = VehicleDynamics::Utilities::GetWheelPhysxHinge(id);
               if (hinge.GetEntityId().IsValid())
               {
                   m_wheelsData.emplace_back(hinge);
               }
           }
       }
       float angular_speed = m_limits.LimitValue(angular.GetZ(), m_limits.m_angularLimit);
       float linear_speed = m_limits.LimitValue(linear.GetX(), m_limits.m_linearLimit);

       AZ::Transform baseTransform;
       AZ::TransformBus::EventResult(baseTransform, this->GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
       auto baseTransformInv = baseTransform.GetInverse();
       for (const auto &wheel : m_wheelsData){
            AZ::Transform wheelTransform;
            AZ::TransformBus::EventResult(wheelTransform, wheel.GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
            auto wheelLocalTransform = baseTransformInv*wheelTransform;
            float wh = wheelLocalTransform.GetTranslation().GetY();
            float vh = (linear_speed - angular_speed * wh) / m_wheelRadius;
            PhysX::JointRequestBus::Event(wheel, &PhysX::JointRequests::SetVelocity,vh);
       }
   }
} // namespace ROS2
