/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2FrameEditorSystemComponent.h"

#include "AzCore/std/containers/vector.h"
#include "NamespaceComputation.h"
#include "ROS2FrameEditorComponent.h"
#include "ROS2FrameSystemBus.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/set.h>
#include <AzCore/std/string/string.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameEditorComponentBus.h>
#include <ROS2/ROS2NamesBus.h>

namespace ROS2
{

    ROS2FrameEditorSystemComponent::ROS2FrameEditorSystemComponent()
    {
        if (ROS2FrameSystemInterface::Get() == nullptr)
        {
            ROS2FrameSystemInterface::Register(this);
        }
    }

    ROS2FrameEditorSystemComponent::~ROS2FrameEditorSystemComponent()
    {
        if (ROS2FrameSystemInterface::Get() == this)
        {
            ROS2FrameSystemInterface::Unregister(this);
        }
    }

    void ROS2FrameEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameEditorSystemComponent, AZ::Component>()->Version(1)->Attribute(
                AZ::Edit::Attributes::SystemComponentTags, AZStd::vector<AZ::Crc32>({ AZ_CRC_CE("AssetBuilder") }));
        }
    }

    void ROS2FrameEditorSystemComponent::Activate()
    {
        AZ_Printf("ROS2FrameSystemComponent", "Activating ROS2FrameSystemComponent");
    }

    void ROS2FrameEditorSystemComponent::Deactivate()
    {
        for (const auto& id : m_registeredEntities)
        {
            AZ::TransformNotificationBus::MultiHandler::BusDisconnect(id);
            AzToolsFramework::EntitySelectionEvents::Bus::MultiHandler::BusDisconnect(id);
        }
        m_registeredEntities.clear();
    }

    AZ::TransformInterface* ROS2FrameEditorSystemComponent::GetEntityTransformInterface(const AZ::Entity* entity)
    {
        if (!entity)
        {
            AZ_Error("GetEntityTransformInterface", false, "Invalid entity!");
            return nullptr;
        }

        auto* interface = entity->FindComponent<AzToolsFramework::Components::TransformComponent>();
        return interface;
    }

    AZStd::set<AZ::EntityId> ROS2FrameEditorSystemComponent::GetChildrenEntityId(const AZ::EntityId& frameEntityId) const
    {
        // get all descendants
        AZStd::vector<AZ::EntityId> children;
        AZ::TransformBus::EventResult(children, frameEntityId, &AZ::TransformBus::Events::GetAllDescendants);
        AZStd::set<AZ::EntityId> childrenWithRos2Frame;
        for (const auto& child : children)
        {
            if (HasROS2FrameComponent(child))
            {
                childrenWithRos2Frame.insert(child);
            }
        }
        return childrenWithRos2Frame;
    }

    bool ROS2FrameEditorSystemComponent::IsTopLevel(const AZ::EntityId& frameEntityId) const
    {
        AZStd::vector<AZ::EntityId> predecessors;
        GetAllAncestorTransformBus(frameEntityId, predecessors);
        const auto superParentId = GetFirstEntityWithROS2FrameComponent(predecessors);
        return (superParentId == frameEntityId);
    }

    AZ::EntityId ROS2FrameEditorSystemComponent::GetParentEntityId(const AZ::EntityId& frameEntityId) const
    {
        AZStd::vector<AZ::EntityId> predecessors;
        GetAllAncestorTransformBus(frameEntityId, predecessors);
        return GetLastEntityWithROS2FrameComponent(predecessors);
    }

    AZStd::vector<AZ::EntityId> ROS2FrameEditorSystemComponent::FindFrameParentPath(AZ::EntityId frameEntityId)
    {
        AZStd::vector<AZ::EntityId> predecessors;
        GetAllAncestorTransformBus(frameEntityId, predecessors);
        return GetEntitiesWithROS2FrameComponent(predecessors);
    }

    void ROS2FrameEditorSystemComponent::RegisterFrame(const AZ::EntityId& frameToRegister)
    {
        m_registeredEntities.insert(frameToRegister);
        AzToolsFramework::EntitySelectionEvents::Bus::MultiHandler::BusConnect(frameToRegister);
        AZ::TransformNotificationBus::MultiHandler::BusConnect(frameToRegister);
    }

    void ROS2FrameEditorSystemComponent::UnregisterFrame(const AZ::EntityId& frameToUnregister)
    {
        AZ::TransformNotificationBus::MultiHandler::BusDisconnect(frameToUnregister);
        AzToolsFramework::EntitySelectionEvents::Bus::MultiHandler::BusDisconnect(frameToUnregister);
        m_registeredEntities.erase(frameToUnregister);
    }

    //! Resolves the ROS 2 frame name based on configuration and entity ID
    AZStd::string ROS2FrameEditorSystemComponent::GetNamespace(const ROS2FrameConfiguration& configuration, AZ::EntityId entity) const
    {
        return ComputeNamespace(configuration, entity, true);
    }

    AZStd::string ROS2FrameEditorSystemComponent::GetFrameName(const ROS2FrameConfiguration& configuration, AZ::EntityId entity) const
    {
        const auto nameSpace = GetNamespace(configuration, entity);
        if (nameSpace.empty())
        {
            return configuration.m_frameName;
        }
        return nameSpace + "/" + configuration.m_frameName;
    }

    AZStd::string ROS2FrameEditorSystemComponent::GetJointName(const ROS2FrameConfiguration& configuration, AZ::EntityId entity) const
    {
        const auto nameSpace = GetNamespace(configuration, entity);
        if (nameSpace.empty())
        {
            return configuration.m_jointName;
        }
        return nameSpace + "/" + configuration.m_frameName;
    }

    void ROS2FrameEditorSystemComponent::OnSelected()
    {
        // find which frame entity was selected
        AZStd::vector<AZ::EntityId> selectedEntityId;
        AzToolsFramework::ToolsApplicationRequests::Bus::BroadcastResult(
            selectedEntityId, &AzToolsFramework::ToolsApplicationRequests::GetSelectedEntities);
        // update
        for (const auto& selectedEntityId : selectedEntityId)
        {
            ROS2FrameEditorComponentBus::Event(selectedEntityId, &ROS2FrameEditorComponentRequests::UpdateNamespace);
        }
    }

    void ROS2FrameEditorSystemComponent::OnParentChanged([[maybe_unused]] AZ::EntityId oldParent, AZ::EntityId newParent)
    {
        AZStd::vector<AZ::EntityId> children;
        AZ::TransformBus::EventResult(children, newParent, &AZ::TransformBus::Events::GetEntityAndAllDescendants);
        // update
        for (const auto& child : children)
        {
            ROS2FrameEditorComponentBus::Event(child, &ROS2FrameEditorComponentRequests::UpdateNamespace);
        }
    }

} // namespace ROS2
