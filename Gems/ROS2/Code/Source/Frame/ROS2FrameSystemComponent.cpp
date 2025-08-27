/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2FrameSystemComponent.h"
#include "AzCore/std/containers/vector.h"
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

    namespace
    {
        [[maybe_unused]] AZStd::string GetName(AZ::EntityId id)
        {
            AZStd::string name;
            AZ::ComponentApplicationBus::BroadcastResult(name, &AZ::ComponentApplicationRequests::GetEntityName, id);
            return name;
        }

        [[maybe_unused]] bool HasRos2FrameComponent(AZ::EntityId id)
        {
            // get entity
            AZ::Entity* entity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, id);
            if (!entity)
            {
                return false;
            }
            auto* componentEditor = entity->FindComponent(AZ::Uuid(ROS2FrameEditorComponentTypeId));
            return componentEditor != nullptr;
        }

        AZStd::vector<AZ::EntityId> GetAllAncestorTransformBus(AZ::EntityId id, AZStd::vector<AZ::EntityId>& predecessors)
        {
            predecessors.push_back(id);
            // Get the transform component
            // get entity
            AZ::Entity* entity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, id);
            if (!entity)
            {
                return predecessors;
            }
            auto* transform = entity->FindComponent<AzToolsFramework::Components::TransformComponent>();
            if (!transform)
            {
                return predecessors;
            }
            AZ::EntityId parentId = transform->GetParentId();
            if (!parentId.IsValid())
            {
                return predecessors;
            }

            return GetAllAncestorTransformBus(parentId, predecessors);
        }

        AZ::EntityId GetOldestAncestorWithRos2FrameComponent(const AZStd::vector<AZ::EntityId>& predecessors)
        {
            for (auto it = predecessors.rbegin(); it != predecessors.rend(); ++it)
            {
                AZ::Entity* entity = nullptr;
                AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, *it);
                if (!entity)
                {
                    continue;
                }
                auto* componentEditor = entity->FindComponent(AZ::Uuid(ROS2FrameEditorComponentTypeId));
                if (componentEditor)
                {
                    return *it;
                }
            }
            return AZ::EntityId();
        }

    } // namespace

    ROS2FrameSystemComponent::ROS2FrameSystemComponent()
    {
        if (ROS2FrameSystemInterface::Get() == nullptr)
        {
            ROS2FrameSystemInterface::Register(this);
        }
    }

    ROS2FrameSystemComponent::~ROS2FrameSystemComponent()
    {
        if (ROS2FrameSystemInterface::Get() == this)
        {
            ROS2FrameSystemInterface::Unregister(this);
        }
    }

    void ROS2FrameSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameSystemComponent, AZ::Component>()->Version(1)->Attribute(
                AZ::Edit::Attributes::SystemComponentTags, AZStd::vector<AZ::Crc32>({ AZ_CRC_CE("AssetBuilder") }));
        }
    }

    void ROS2FrameSystemComponent::Activate()
    {
        AZ_Printf("ROS2FrameSystemComponent", "Activating ROS2FrameSystemComponent");
    }

    void ROS2FrameSystemComponent::Deactivate()
    {
        for (const auto& id : m_registeredEntities)
        {
            AZ::TransformNotificationBus::MultiHandler::BusDisconnect(id);
            AzToolsFramework::EntitySelectionEvents::Bus::MultiHandler::BusDisconnect(id);
        }
        m_registeredEntities.clear();
    }

    AZ::TransformInterface* ROS2FrameSystemComponent::GetEntityTransformInterface(const AZ::Entity* entity)
    {
        if (!entity)
        {
            AZ_Error("GetEntityTransformInterface", false, "Invalid entity!");
            return nullptr;
        }

        auto* interface = entity->FindComponent<AzToolsFramework::Components::TransformComponent>();
        return interface;
    }

    AZStd::set<AZ::EntityId> ROS2FrameSystemComponent::GetChildrenEntityId(const AZ::EntityId& frameEntityId) const
    {
        // get all descendants
        AZStd::vector<AZ::EntityId> children;
        AZ::TransformBus::EventResult(children, frameEntityId, &AZ::TransformBus::Events::GetAllDescendants);
        AZStd::set<AZ::EntityId> childrenWithRos2Frame;
        for (const auto& child : children)
        {
            if (HasRos2FrameComponent(child))
            {
                childrenWithRos2Frame.insert(child);
            }
        }
        return childrenWithRos2Frame;
    }

    bool ROS2FrameSystemComponent::IsTopLevel(const AZ::EntityId& frameEntityId) const
    {
        AZStd::vector<AZ::EntityId> predecessors;
        GetAllAncestorTransformBus(frameEntityId, predecessors);
        const auto superParentId = GetOldestAncestorWithRos2FrameComponent(predecessors);
        return (superParentId == frameEntityId);
    }

    AZ::EntityId ROS2FrameSystemComponent::GetParentEntityId(const AZ::EntityId& frameEntityId) const
    {
        AZStd::vector<AZ::EntityId> predecessors;
        GetAllAncestorTransformBus(frameEntityId, predecessors);
        return GetOldestAncestorWithRos2FrameComponent(predecessors);
    }

    AZStd::vector<AZ::EntityId> ROS2FrameSystemComponent::FindFrameParentPath(AZ::EntityId frameEntityId)
    {

        return {};
    }

    void ROS2FrameSystemComponent::RegisterFrame(const AZ::EntityId& frameToRegister)
    {
        m_registeredEntities.insert(frameToRegister);
        AzToolsFramework::EntitySelectionEvents::Bus::MultiHandler::BusConnect(frameToRegister);
        AZ::TransformNotificationBus::MultiHandler::BusConnect(frameToRegister);
    }

    void ROS2FrameSystemComponent::UnregisterFrame(const AZ::EntityId& frameToUnregister)
    {
        AZ::TransformNotificationBus::MultiHandler::BusDisconnect(frameToUnregister);
        AzToolsFramework::EntitySelectionEvents::Bus::MultiHandler::BusDisconnect(frameToUnregister);
        m_registeredEntities.erase(frameToUnregister);
    }

    //! Resolves the ROS 2 frame name based on configuration and entity ID
    AZStd::string ROS2FrameSystemComponent::GetNamespace(const ROS2FrameConfiguration& configuration, AZ::EntityId entity) const
    {
        // An empty, blank namespace
        if (configuration.m_namespaceConfiguration.GetNamespaceStrategy() == NamespaceConfiguration::NamespaceStrategy::Empty)
        {
            return "";
        }

        // Non-empty and based on user-provided value.
        if (configuration.m_namespaceConfiguration.GetNamespaceStrategy() == NamespaceConfiguration::NamespaceStrategy::Custom)
        {
            return configuration.m_namespaceConfiguration.GetCustomNamespace();
        }

        AZStd::string thisEntityName;

        AZ::ComponentApplicationBus::BroadcastResult(thisEntityName, &AZ::ComponentApplicationRequests::GetEntityName, entity);
        ROS2NamesRequestBus::BroadcastResult(thisEntityName, &ROS2NamesRequests::RosifyName, thisEntityName);

        // Generate from Entity name, but substitute disallowed characters through RosifyName.
        if (configuration.m_namespaceConfiguration.GetNamespaceStrategy() == NamespaceConfiguration::NamespaceStrategy::FromEntityName)
        {
            return thisEntityName;
        }

        // FromEntityName for top-level frames, Empty otherwise.
        if (configuration.m_namespaceConfiguration.GetNamespaceStrategy() == NamespaceConfiguration::NamespaceStrategy::Default)
        {
            AZStd::vector<AZ::EntityId> predecessors;
            GetAllAncestorTransformBus(entity, predecessors);

            const auto superParentId = GetOldestAncestorWithRos2FrameComponent(predecessors);
            const bool is_root = (superParentId == entity);
            if (is_root)
            {
                return thisEntityName;
            }

            AZStd::string superParentName;
            AZ::ComponentApplicationBus::BroadcastResult(superParentName, &AZ::ComponentApplicationRequests::GetEntityName, superParentId);

            ROS2NamesRequestBus::BroadcastResult(superParentName, &ROS2NamesRequests::RosifyName, superParentName);
            return superParentName;
        }
        return "";
    }

    AZStd::string ROS2FrameSystemComponent::GetFrameName(const ROS2FrameConfiguration& configuration, AZ::EntityId entity) const
    {
        const auto nameSpace = GetNamespace(configuration, entity);
        if (nameSpace.empty())
        {
            return configuration.m_frameName;
        }
        return nameSpace + "/" + configuration.m_frameName;
    }

    AZStd::string ROS2FrameSystemComponent::GetJointName(const ROS2FrameConfiguration& configuration, AZ::EntityId entity) const
    {
        const auto nameSpace = GetNamespace(configuration, entity);
        if (nameSpace.empty())
        {
            return configuration.m_jointName;
        }
        return nameSpace + "/" + configuration.m_frameName;
    }

    void ROS2FrameSystemComponent::OnSelected()
    {
        // find which frame entity was selected
        AZStd::vector<AZ::EntityId> selectedEntityId;
        AzToolsFramework::ToolsApplicationRequests::Bus::BroadcastResult(
            selectedEntityId, &AzToolsFramework::ToolsApplicationRequests::GetSelectedEntities);
        // update
        for (const auto& selectedEntityId : selectedEntityId)
        {
            ROS2FrameEditorComponentBus::Event(selectedEntityId, &ROS2FrameEditorComponentRequests::UpdateNamespace, "");
        }
    }

    void ROS2FrameSystemComponent::OnParentChanged([[maybe_unused]] AZ::EntityId oldParent, AZ::EntityId newParent)
    {
        AZStd::vector<AZ::EntityId> children;
        AZ::TransformBus::EventResult(children, newParent, &AZ::TransformBus::Events::GetEntityAndAllDescendants);
        // update
        for (const auto& child : children)
        {
            ROS2FrameEditorComponentBus::Event(child, &ROS2FrameEditorComponentRequests::UpdateNamespace, "");
        }
    }

} // namespace ROS2
