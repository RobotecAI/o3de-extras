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
        AZStd::string GetName(AZ::EntityId id)
        {
            AZStd::string name;
            AZ::ComponentApplicationBus::BroadcastResult(name, &AZ::ComponentApplicationRequests::GetEntityName, id);
            return name;
        }

        AZStd::vector<AZ::EntityId> GetAllAncestorTransformBus(AZ::EntityId id, AZStd::vector<AZ::EntityId>& predecessors)
        {
            predecessors.push_back(id);
            // Get the transform component
            // get entity
            AZ::Entity * entity = nullptr;
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
                AZ::Entity * entity = nullptr;
                AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, *it);
                auto * componentEditor = entity->FindComponent(AZ::Uuid(ROS2FrameEditorComponentTypeId));
                if (componentEditor)
                {
                    return *it;
                }
            }
            return AZ::EntityId();
        }
    }


    void ROS2FrameSystemComponent::CanParentChange(bool &parentCanChange, AZ::EntityId oldParent, AZ::EntityId newParent)
    {
        (void)parentCanChange; (void)oldParent; (void)newParent;
        // get all children of the old parent
        AZStd::vector<AZ::EntityId> children;
        AZ::TransformBus::EventResult(children, oldParent, &AZ::TransformInterface::GetAllDescendants);
        for (const auto& predecessor : children)
        {
            m_dirtyFrames.insert(predecessor);
        }
    }

    void ROS2FrameSystemTransformHandler::AddFrameEntity(AZ::EntityId frameEntityId)
    {
        if (!m_frameEntities.contains(frameEntityId))
        {
            m_frameEntities.insert(frameEntityId);
        }
    }
    void ROS2FrameSystemTransformHandler::RemoveFrameEntity(AZ::EntityId frameEntityId)
    {
        if (m_frameEntities.contains(frameEntityId))
        {
            m_frameEntities.erase(frameEntityId);
        }
    }

    unsigned int ROS2FrameSystemTransformHandler::GetFrameCount()
    {
        return m_frameEntities.size();
    }

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
        AZ::TickBus::Handler::BusConnect();
    }

    void ROS2FrameSystemComponent::Deactivate()
    {
        AZ::TransformNotificationBus::MultiHandler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
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

    bool ROS2FrameSystemComponent::IsTopLevel(const AZ::EntityId& frameEntityId) const
    {
        AZ_Assert(false, "Not implemented yet");
        return false;
    }

    AZ::EntityId ROS2FrameSystemComponent::GetParentEntityId(const AZ::EntityId& frameEntityId) const
    {
        AZ_Assert(false, "Not implemented yet");
        return AZ::EntityId();
    }

    AZStd::vector<AZ::EntityId> ROS2FrameSystemComponent::FindFrameParentPath(AZ::EntityId frameEntityId)
    {
        AZ_Assert(false, "Find Parent path Not implemented yet");
        return {};
    }

    void ROS2FrameSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        for (const auto id : m_dirtyFrames)
        {
            AZ_Printf("ROS2FrameSystemComponent", "Processing dirty frame for entity %s", GetName(id).c_str());
            ROS2::ROS2FrameEditorComponentBus::Event(id, &ROS2::ROS2FrameEditorComponentRequests::UpdateNamespace, "");
        }
        m_dirtyFrames.clear();
    }


    void ROS2FrameSystemComponent::RegisterFrame(const AZ::EntityId& frameToRegister)
    {
        AZ::TransformNotificationBus::MultiHandler::BusConnect(frameToRegister);
        AZStd::vector<AZ::EntityId> predecessors;
        GetAllAncestorTransformBus(frameToRegister, predecessors);
        for (const auto& predecessor : predecessors)
        {
            AZ::TransformNotificationBus::MultiHandler::BusConnect(predecessor);
        }
    }

    void ROS2FrameSystemComponent::UnregisterFrame(const AZ::EntityId& frameToUnregister)
    {
        AZ::TransformNotificationBus::MultiHandler::BusDisconnect(frameToUnregister);
    }

    void ROS2FrameSystemComponent::MoveFrameDetach(
        const AZ::EntityId& frameEntityId, const AZStd::set<AZ::EntityId>& newPathToParentFrameSet)
    {
        AZ_Warning("ROS2FrameSystemComponent", false, "Not implemented yet");
    }

    void ROS2FrameSystemComponent::MoveFrameAttach(
        const AZ::EntityId& frameEntityId, const AZ::EntityId& newFrameParent, const AZStd::vector<AZ::EntityId>& newPathToParentFrame)
    {
        AZ_Warning("ROS2FrameSystemComponent", false, "Not implemented yet");
    }

    void ROS2FrameSystemComponent::MoveFrame(const AZ::EntityId& frameEntityId, const AZ::EntityId& newParent)
    {
        AZ_Warning("ROS2FrameSystemComponent", false, "Not implemented yet");
    }

    void ROS2FrameSystemComponent::UpdateNamespaces(AZ::EntityId frameEntity, AZ::EntityId frameParentEntity, bool isActive)
    {
        AZ_Warning("ROS2FrameSystemComponent", false, "UpdateNamespaces1 Not implemented yet");
    }

    void ROS2FrameSystemComponent::UpdateNamespaces(AZ::EntityId frameEntity, AZStd::string parentNamespace, bool isActive)
    {
        AZ_Warning("ROS2FrameSystemComponent", false, "UpdateNamespaces2 Not implemented yet");
    }

    void ROS2FrameSystemComponent::NotifyChange(const AZ::EntityId& frameEntityId)
    {
        AZStd::vector<AZ::EntityId> children;
        AZ::TransformBus::EventResult(children, frameEntityId, &AZ::TransformBus::Events::GetEntityAndAllDescendants);
        for (const auto& child : children)
        {
            ROS2::ROS2FrameEditorComponentBus::Event(
                child, &ROS2::ROS2FrameEditorComponentRequests::UpdateNamespace, "");
        }
    }

    AZStd::set<AZ::EntityId> ROS2FrameSystemComponent::GetChildrenEntityId(const AZ::EntityId& frameEntityId) const
    {
        AZ_Warning("ROS2FrameSystemComponent", false, "GetAllPredecessors Not implemented yet");
        return {};
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
        return nameSpace +"/"+ configuration.m_frameName;
    }
} // namespace ROS2
