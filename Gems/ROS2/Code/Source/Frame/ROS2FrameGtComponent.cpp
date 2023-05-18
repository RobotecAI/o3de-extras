/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Frame/ROS2FrameGtComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Names.h>

#include "ROS2FrameUtils.h"

namespace ROS2
{
    void ROS2FrameGtComponent::Activate()
    {
        m_namespaceConfiguration.PopulateNamespace(true, GetEntity()->GetName());

        AZ_TracePrintf(
            "ROS2FrameGtComponent",
            "Setting up dynamic transform between reference %s and target %s to be published continuously to /tf\n",
            GetGlobalFrameName().data(),
            GetFrameID().data());

        m_ros2Transform = AZStd::make_unique<ROS2Transform>(GetGlobalFrameName(), GetFrameID(), true);
        auto* transformInterface = Internal::GetEntityTransformInterface(GetEntity());
        // TODO: perhaps inverse ?
        auto worldFromReference = transformInterface->GetWorldTM();
        m_referenceFromWorld = worldFromReference.GetInverse();
        // TODO: perhaps static publish needed here of map_gt?
        //        m_ros2Transform->Publish(GetFrameTransform());
        AZ::TickBus::Handler::BusConnect();
    }

    void ROS2FrameGtComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        m_ros2Transform.reset();
    }

    void ROS2FrameGtComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        m_ros2Transform->Publish(GetFrameTransform());
    }

    AZStd::string ROS2FrameGtComponent::GetGlobalFrameName() const
    {
        return ROS2Names::GetNamespacedName(GetNamespace(), m_referenceFrameName);
    }

    AZ::Transform ROS2FrameGtComponent::GetFrameTransform() const
    {
        auto* transformInterface = Internal::GetEntityTransformInterface(GetEntity());
        const auto worldFromThis = transformInterface->GetWorldTM();
        return m_referenceFromWorld * worldFromThis;
    }

    AZStd::string ROS2FrameGtComponent::GetFrameID() const
    {
        return ROS2Names::GetNamespacedName(GetNamespace(), m_frameName);
    }

    AZStd::string ROS2FrameGtComponent::GetNamespace() const
    {
        AZStd::string parentNamespace;
        return m_namespaceConfiguration.GetNamespace(parentNamespace);
    }

    void ROS2FrameGtComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameGtComponent, AZ::Component>()
                ->Version(1)
                ->Field("Namespace Configuration", &ROS2FrameGtComponent::m_namespaceConfiguration)
                ->Field("Reference Frame Name", &ROS2FrameGtComponent::m_referenceFrameName)
                ->Field("Frame Name", &ROS2FrameGtComponent::m_frameName);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2FrameGtComponent>("ROS2 Frame GT", "[ROS2 Frame GT component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2FrameGtComponent::m_namespaceConfiguration,
                        "Namespace Configuration",
                        "Namespace Configuration")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2FrameGtComponent::m_referenceFrameName,
                        "Reference Frame Name",
                        "Reference Frame Name")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2FrameGtComponent::m_frameName, "Frame Name", "Frame Name");
            }
        }
    }

    void ROS2FrameGtComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2FrameGt"));
    }

    void ROS2FrameGtComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2FrameGt"));
    }

    void ROS2FrameGtComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    ROS2FrameGtComponent::ROS2FrameGtComponent() = default;

    ROS2FrameGtComponent::ROS2FrameGtComponent(const AZStd::string& frameId)
        : m_frameName(frameId)
    {
    }
} // namespace ROS2
