/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "NamespaceComputation.h"

#include "AzCore/Component/ComponentApplicationBus.h"
#include "AzCore/Component/Entity.h"
#include "AzCore/Component/TransformBus.h"
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
                const auto* intreface = dynamic_cast<const ROSFrameInterface *>(componentEditor);
                if (intreface)
                {
                    return intreface->GetConfiguration();
                }
            }
            const auto* componentGame = entity->FindComponent(AZ::Uuid(ROS2FrameComponentTypeId));
            if (componentGame)
            {
                const auto* intreface = dynamic_cast<const ROSFrameInterface *>(componentGame);
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
            return ;
        }
        auto* transformInterface = entity->GetTransform();
        if (!transformInterface)
        {
            return ;
        }
        AZ::EntityId parentId = transformInterface->GetParentId();
        if (!parentId.IsValid())
        {
            return ;
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

    AZStd::string ComputeNamespace(const ROS2FrameConfiguration& configuration, AZ::EntityId entity, bool debug)
    {
        if (configuration.m_namespaceConfiguration.m_namespaceStrategy == NamespaceConfiguration::NamespaceStrategy::Empty)
        {
            return "";
        }

        // Non-empty and based on user-provided value.
        if (configuration.m_namespaceConfiguration.m_namespaceStrategy == NamespaceConfiguration::NamespaceStrategy::Custom)
        {
            return configuration.m_namespaceConfiguration.m_customNamespace;
        }

        AZStd::string thisEntityName;

        AZ::ComponentApplicationBus::BroadcastResult(thisEntityName, &AZ::ComponentApplicationRequests::GetEntityName, entity);
        ROS2NamesRequestBus::BroadcastResult(thisEntityName, &ROS2NamesRequests::RosifyName, thisEntityName);

        // Generate from Entity name, but substitute disallowed characters through RosifyName.
        if (configuration.m_namespaceConfiguration.m_namespaceStrategy == NamespaceConfiguration::NamespaceStrategy::FromEntityName)
        {
            return thisEntityName;
        }

        // FromEntityName for top-level frames, Empty otherwise.
        if (configuration.m_namespaceConfiguration.m_namespaceStrategy == NamespaceConfiguration::NamespaceStrategy::Default)
        {
            const AZStd::vector<AZ::EntityId> predecessors = GetAllAncestorTransformBus(entity);
            const AZStd::vector<AZ::EntityId> predecessorsWithRos2Frame = GetEntitiesWithROS2FrameComponent(predecessors);
            const auto topLevelParentId = predecessorsWithRos2Frame.empty() ? AZ::EntityId() : predecessorsWithRos2Frame.back();
            if (debug)
            {
                AZ_Printf("ROS2FrameSystemComponent::GetNamespace", "Resolving namespace for entity %s", thisEntityName.c_str());
                for (auto it = predecessors.rbegin(); it != predecessors.rend(); ++it)
                {
                    const auto entityId = *it;
                    const auto thisFrameConfig = GetConfigurationFromComponent(entityId);
                    AZStd::string thisStrategyName = "No ROS2FrameComponent";
                    if (thisFrameConfig)
                    {
                        thisStrategyName = strategyToString.at(thisFrameConfig->m_namespaceConfiguration.m_namespaceStrategy);
                    }
                    AZ_Printf(
                        "ROS2FrameSystemComponent::GetNamespace",
                        " - %s  [%s] %s",
                        GetName(entityId).c_str(),
                        thisStrategyName.c_str(),
                        ((topLevelParentId == entityId) ? " (ros2 root frame)" : ""));
                }
            }
            if (predecessorsWithRos2Frame.empty())
            {
                // No ROS2Frame components in ancestors, so we are top-level.
                return thisEntityName;
            }

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
            if (debug)
            {
                AZ_Printf(
                    "ROS2FrameSystemComponent::GetNamespace",
                    "Resolved namespace %s for entity %s",
                    resolvedName.c_str(),
                    thisEntityName.c_str());
            }
            return resolvedName;
        }

        return "";
    }
} // namespace ROS2