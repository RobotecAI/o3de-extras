/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SpawnerComponentController.h"
#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Debug/Trace.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Spawner/SpawnerInfo.h>

namespace ROS2
{
    namespace
    {
        //! Warns when a deprecated Spawner configuration is read from a level or a prefab.
        //! OnWriteEnd is also called when objects are cloned, hence the warning is reported only once.
        class ROS2SpawnerComponentConfigEventHandler : public AZ::SerializeContext::IEventHandler
        {
        public:
            void OnWriteEnd([[maybe_unused]] void* classPtr) override
            {
                AZ_WarningOnce(
                    "ROS2SpawnerComponent",
                    false,
                    "A deprecated ROS2 Spawner component was loaded from a level or a prefab. %s",
                    SpawnerRemovedMessage);
            }
        };
    } // namespace

    void ROS2SpawnerServiceNames::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2SpawnerServiceNames>()
                ->Version(1)
                ->Field("Get available spawnable names service name", &ROS2SpawnerServiceNames::m_availableSpawnableNamesServiceName)
                ->Field("Spawn entity service name", &ROS2SpawnerServiceNames::m_spawnEntityServiceName)
                ->Field("Delete entity service name", &ROS2SpawnerServiceNames::m_deleteEntityServiceName)
                ->Field("Get spawn point info service name", &ROS2SpawnerServiceNames::m_spawnPointInfoServiceName)
                ->Field("Get spawn points names service name", &ROS2SpawnerServiceNames::m_spawnPointsNamesServiceName);

            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<ROS2SpawnerServiceNames>("ROS2SpawnerServiceNames", "Service names for ROS2SpawnerComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2SpawnerServiceNames::m_availableSpawnableNamesServiceName,
                        "Get available spawnable names service name",
                        "Service name for getting available spawnable names")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2SpawnerServiceNames::m_spawnEntityServiceName,
                        "Spawn entity service name",
                        "Service name for spawning entity")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2SpawnerServiceNames::m_deleteEntityServiceName,
                        "Delete entity service name",
                        "Service name for deleting entity")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2SpawnerServiceNames::m_spawnPointInfoServiceName,
                        "Get spawn point info service name",
                        "Service name for getting spawn point info")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2SpawnerServiceNames::m_spawnPointsNamesServiceName,
                        "Get spawn points names service name",
                        "Service name for getting spawn points names");
            }
        }
    }

    void ROS2SpawnerComponentConfig::Reflect(AZ::ReflectContext* context)
    {
        ROS2SpawnerServiceNames::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2SpawnerComponentConfig, AZ::ComponentConfig>()
                ->Version(1)
                ->EventHandler<ROS2SpawnerComponentConfigEventHandler>()
                ->Field("Editor entity id", &ROS2SpawnerComponentConfig::m_editorEntityId)
                ->Field("Support WGS", &ROS2SpawnerComponentConfig::m_supportWGS)
                ->Field("Spawnables", &ROS2SpawnerComponentConfig::m_spawnables)
                ->Field("Default spawn pose", &ROS2SpawnerComponentConfig::m_defaultSpawnPose)
                ->Field("Service names", &ROS2SpawnerComponentConfig::m_serviceNames)
                ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);

            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<ROS2SpawnerComponentConfig>("ROS2SpawnerComponentConfig", "Config for ROS2SpawnerComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2SpawnerComponentConfig::m_supportWGS, "Support WGS", "Support for spawning entities using WGS84 coordinate system")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2SpawnerComponentConfig::m_spawnables, "Spawnables", "Spawnables")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2SpawnerComponentConfig::m_defaultSpawnPose,
                        "Default spawn pose",
                        "Default spawn pose")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &ROS2SpawnerComponentConfig::m_serviceNames, "Service names", "Service names");
            }
        }
    }

    const AZ::Transform& ROS2SpawnerComponentController::GetDefaultSpawnPose() const
    {
        AZ_Error("ROS2SpawnerComponent", false, "GetDefaultSpawnPose: %s", SpawnerRemovedMessage);
        return m_config.m_defaultSpawnPose;
    }

    AZStd::unordered_map<AZStd::string, SpawnPointInfo> ROS2SpawnerComponentController::GetAllSpawnPointInfos() const
    {
        AZ_Error("ROS2SpawnerComponent", false, "GetAllSpawnPointInfos: %s", SpawnerRemovedMessage);
        return {};
    }

    void ROS2SpawnerComponentController::Reflect(AZ::ReflectContext* context)
    {
        ROS2SpawnerComponentConfig::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2SpawnerComponentController>()->Version(1)->Field(
                "Configuration", &ROS2SpawnerComponentController::m_config);

            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext->Class<ROS2SpawnerComponentController>("ROS2SpawnerComponentController", "Controller for ROS2SpawnerComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "Manages spawning of robots in configurable locations")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2SpawnerComponentController::m_config)
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    void ROS2SpawnerComponentController::Init()
    {
    }

    void ROS2SpawnerComponentController::Activate(AZ::EntityId entityId)
    {
        m_config.m_editorEntityId = entityId;
        SpawnerRequestsBus::Handler::BusConnect(entityId);
    }

    void ROS2SpawnerComponentController::Deactivate()
    {
        SpawnerRequestsBus::Handler::BusDisconnect();
    }

    ROS2SpawnerComponentController::ROS2SpawnerComponentController(const ROS2SpawnerComponentConfig& config)
    {
        SetConfiguration(config);
    }

    void ROS2SpawnerComponentController::SetConfiguration(const ROS2SpawnerComponentConfig& config)
    {
        m_config = config;
    }

    const ROS2SpawnerComponentConfig& ROS2SpawnerComponentController::GetConfiguration() const
    {
        return m_config;
    }

} // namespace ROS2
