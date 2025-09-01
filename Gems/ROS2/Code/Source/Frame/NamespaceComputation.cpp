/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "NamespaceComputation.h"

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>
#include <ROS2/ROS2NamesBus.h>

namespace ROS2
{

    AZStd::string GetName(AZ::EntityId id)
    {
        AZStd::string name;
        AZ::ComponentApplicationBus::BroadcastResult(name, &AZ::ComponentApplicationRequests::GetEntityName, id);
        return name;
    }

    AZStd::optional<ROS2FrameConfiguration> GetConfigurationFromComponent(AZ::EntityId id)
    {
        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, id);
        if (!entity)
        {
            return ROS2FrameConfiguration();
        }
        const auto* componentEditor = entity->FindComponent(AZ::Uuid(ROS2FrameEditorComponentTypeId));
        if (componentEditor)
        {
            const auto* intreface = dynamic_cast<const ROSFrameInterface*>(componentEditor);
            if (intreface)
            {
                return intreface->GetConfiguration();
            }
        }
        const auto* componentGame = entity->FindComponent(AZ::Uuid(ROS2FrameComponentTypeId));
        if (componentGame)
        {
            const auto* intreface = dynamic_cast<const ROSFrameInterface*>(componentGame);
            if (intreface)
            {
                return intreface->GetConfiguration();
            }
        }
        return AZStd::nullopt;
    }

    const AZStd::unordered_map<NamespaceConfiguration::NamespaceStrategy, AZStd::string_view> strategyToString = {
        { NamespaceConfiguration::NamespaceStrategy::Default, "Default" },
        { NamespaceConfiguration::NamespaceStrategy::Empty, "Empty" },
        { NamespaceConfiguration::NamespaceStrategy::FromEntityName, "FromEntityName" },
        { NamespaceConfiguration::NamespaceStrategy::Custom, "Custom" },
    };

    bool HasROS2FrameComponent(AZ::EntityId id)
    {
        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, id);
        if (!entity)
        {
            return false;
        }
        if (entity->FindComponent(AZ::Uuid(ROS2FrameEditorComponentTypeId)))
        {
            return true;
        }
        if (entity->FindComponent(AZ::Uuid(ROS2FrameComponentTypeId)))
        {
            return true;
        }
        return false;
    }

    void TraverseTransforms(AZ::EntityId id, AZStd::vector<AZ::EntityId>& predecessors)
    {
        predecessors.push_back(id);
        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, id);
        if (!entity)
        {
            return;
        }
        auto* transformInterface = entity->GetTransform();
        if (!transformInterface)
        {
            return;
        }
        AZ::EntityId parentId = transformInterface->GetParentId();
        if (!parentId.IsValid())
        {
            return;
        }
        TraverseTransforms(parentId, predecessors);
    }

    AZStd::vector<AZ::EntityId> GetAllAncestorTransformBus(const AZ::EntityId& id)
    {
        AZStd::vector<AZ::EntityId> predecessors;
        TraverseTransforms(id, predecessors);
        return predecessors;
    }

    AZStd::vector<AZ::EntityId> GetEntitiesWithROS2FrameComponent(const AZStd::vector<AZ::EntityId>& unfiletered)
    {
        AZStd::vector<AZ::EntityId> filtered;
        for (auto it = unfiletered.begin(); it != unfiletered.end(); ++it)
        {
            AZ::Entity* entity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, *it);
            if (!entity)
            {
                continue;
            }
            if (HasROS2FrameComponent(*it))
            {
                filtered.push_back(*it);
            }
        }
        return filtered;
    }

    AZ::EntityId GetFirstEntityWithROS2FrameComponent(const AZStd::vector<AZ::EntityId>& predecessors)
    {
        for (auto it = predecessors.rbegin(); it != predecessors.rend(); ++it)
        {
            if (HasROS2FrameComponent(*it))
            {
                return *it;
            }
        }
        return AZ::EntityId();
    }

    AZ::EntityId GetLastEntityWithROS2FrameComponent(const AZStd::vector<AZ::EntityId>& predecessors)
    {
        for (auto it = predecessors.begin(); it != predecessors.end(); ++it)
        {
            if (HasROS2FrameComponent(*it))
            {
                return *it;
            }
        }
        return AZ::EntityId();
    }

    AZStd::string GetNamespacedName(const AZStd::string& namespaceName, const AZStd::string& name)
    {
        if (namespaceName.empty())
        {
            return name;
        }
        return namespaceName + "/" + name;
    }

    AZStd::string ComputeNamespace(const ROS2FrameConfiguration& configuration, AZ::EntityId entity)
    {
        // Compute namespace based on ancestor frames.
        const AZStd::vector<AZ::EntityId> predecessors = GetAllAncestorTransformBus(entity);
        const AZStd::vector<AZ::EntityId> predecessorsWithRos2Frame = GetEntitiesWithROS2FrameComponent(predecessors);
        const auto topLevelParentId = predecessorsWithRos2Frame.empty() ? AZ::EntityId() : predecessorsWithRos2Frame.back();

        const AZStd::string topLevelParentName = GetName(topLevelParentId);

        AZStd::string resolvedName = "";
        for (auto it = predecessorsWithRos2Frame.rbegin(); it != predecessorsWithRos2Frame.rend(); ++it)
        {
            const auto ancestorId = *it;
            const auto ancestorConfig = GetConfigurationFromComponent(ancestorId);
            if (!ancestorConfig)
            {
                continue;
            }
            const auto ancestorStrategy = ancestorConfig->m_namespaceConfiguration.m_namespaceStrategy;
            if (ancestorStrategy == NamespaceConfiguration::NamespaceStrategy::FromEntityName)
            {
                if (ancestorId == topLevelParentId)
                {
                    resolvedName = GetName(ancestorId);
                }
                else
                {
                    resolvedName += "/" + GetName(ancestorId);
                }
            }
            else if (ancestorStrategy == NamespaceConfiguration::NamespaceStrategy::Custom)
            {
                if (ancestorId == topLevelParentId)
                {
                    resolvedName = ancestorConfig->m_namespaceConfiguration.m_customNamespace;
                }
                else
                {
                    resolvedName += "/" + ancestorConfig->m_namespaceConfiguration.m_customNamespace;
                }
            }
            else if (ancestorStrategy == NamespaceConfiguration::NamespaceStrategy::Empty)
            {
                void(); // do nothing
            }
            else if (ancestorStrategy == NamespaceConfiguration::NamespaceStrategy::Default)
            {
                if (ancestorId == topLevelParentId)
                {
                    resolvedName = GetName(ancestorId);
                }
            }
        }

        return resolvedName;
    }
} // namespace ROS2
