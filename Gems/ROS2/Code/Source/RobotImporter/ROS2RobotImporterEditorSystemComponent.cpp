/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2RobotImporterEditorSystemComponent.h"
#include "AzCore/std/string/string.h"
#include "Bus/RobotImporterBus.h"
#include "RobotImporter/URDF/UrdfParser.h"
#include "RobotImporterWidget.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <AzToolsFramework/API/ViewPaneOptions.h>
#if !defined(Q_MOC_RUN)
#include <QWindow>
#endif
#include "RobotImporter/Utils/FilePath.h"
namespace ROS2
{
    void ROS2RobotImporterEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        RobotImporterRequestHandler::Reflect(context);
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2RobotImporterEditorSystemComponent, ROS2RobotImporterSystemComponent>()->Version(0);
        }

    }

    void ROS2RobotImporterEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        ROS2RobotImporterSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ROS2RobotImporterEditorService"));
    }

    void ROS2RobotImporterEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        ROS2RobotImporterSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ROS2RobotImporterEditorService"));
    }

    void ROS2RobotImporterEditorSystemComponent::Activate()
    {
        ROS2RobotImporterSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
        RobotImporterRequestBus::Handler::BusConnect();
    }

    void ROS2RobotImporterEditorSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        RobotImporterRequestBus::Handler::BusDisconnect();
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        ROS2RobotImporterSystemComponent::Deactivate();
    }

    void ROS2RobotImporterEditorSystemComponent::NotifyRegisterViews()
    {
        AzToolsFramework::ViewPaneOptions options;
        options.showOnToolsToolbar = true;
        options.isDockable = false;
        options.detachedWindow = true;
        options.canHaveMultipleInstances = false;
        options.isDisabledInSimMode = true;
        options.isDeletable = true;

        options.toolbarIcon = ":/ROS2/ROS_import_icon.svg";
        AzToolsFramework::RegisterViewPane<RobotImporterWidget>("Robot Importer", "ROS2", options);
    }

    void ROS2RobotImporterEditorSystemComponent::GeneratePrefabFromFile(
        const AZStd::string_view filePath, bool importAssetWithUrdf, bool useArticulation)
    {
        if (m_importState != UrdfImportState::Done && m_importState != UrdfImportState::Failed)
        {
            AZ_Warning("ROS2EditorSystemComponent", false, "Import already in progress");
            return;
        }
        m_importedPath = filePath;
        m_importAssetWithUrdf = importAssetWithUrdf;
        m_useArticulation = useArticulation;
        m_importState = UrdfImportState::In_Progress;
        AZ::TickBus::Handler::BusConnect();
    }

    void ROS2RobotImporterEditorSystemComponent::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        AZ_Printf("ROS2EditorSystemComponent", "ROS2EditorSystemComponent::OnTick");
        if (m_importState == UrdfImportState::In_Progress)
        {
            AZ_Printf("ROS2EditorSystemComponent", "ROS2EditorSystemComponent::OnTick::In_Progress");
            if (m_importedPath.empty())
            {
                AZ_Warning("ROS2EditorSystemComponent", false, "Empty file path");
                m_importState = UrdfImportState::Failed;
            }
            if (Utils::IsFileXacro(m_importedPath))
            {
                AZ_Warning("ROS2EditorSystemComponent", false, "XACRO is not supported");
                m_importState = UrdfImportState::Failed;
            }
            m_parsedUrdf = UrdfParser::ParseFromFile(m_importedPath);
            if (!m_parsedUrdf)
            {
                AZ_Warning("ROS2EditorSystemComponent", false, "URDF parsing failed");
                const auto log = UrdfParser::GetUrdfParsingLog();
                AZ_Warning("ROS2EditorSystemComponent", false, "%s", log.c_str());
                m_importState = UrdfImportState::Failed;
            }
            m_importState = UrdfImportState::In_Progress_Copy_Assets;
            return;
        }

        if (m_importState == UrdfImportState::In_Progress_Copy_Assets)
        {
            AZ_Printf("ROS2EditorSystemComponent", "ROS2EditorSystemComponent::OnTick::In_Progress_Copy_Assets");
            auto collidersNames = Utils::GetMeshesFilenames(m_parsedUrdf->getRoot(), false, true);
            auto visualNames = Utils::GetMeshesFilenames(m_parsedUrdf->getRoot(), true, false);
            auto meshNames = Utils::GetMeshesFilenames(m_parsedUrdf->getRoot(), true, true);
            m_urdfAssetsMapping = AZStd::make_shared<Utils::UrdfAssetMap>(
                Utils::CopyAssetForURDFAndCreateAssetMap(meshNames, m_importedPath, collidersNames, visualNames));
            m_importState = UrdfImportState::In_Progress_Wait_For_AP;
            return;
        }

        if (m_importState == UrdfImportState::In_Progress_Wait_For_AP)
        {
            AZ_Printf("ROS2EditorSystemComponent", "ROS2EditorSystemComponent::OnTick::In_Progress_Wait_For_AP");
            bool allAssetProcessed = false;

            AZStd::this_thread::sleep_for(AZStd::chrono::milliseconds(300));
            allAssetProcessed = true;
            for (const auto& [name, asset] : *m_urdfAssetsMapping)
            {
                auto sourceAssetFullPath = asset.m_availableAssetInfo.m_sourceAssetGlobalPath;
                if (sourceAssetFullPath.empty())
                {
                    AZ_Printf("FooTest", "asset %s is has no sourceAssetFullPath", name.c_str());
                    continue;
                }
                using namespace AzToolsFramework;
                using namespace AzToolsFramework::AssetSystem;
                AZ::Outcome<AssetSystem::JobInfoContainer> result = AZ::Failure();
                AssetSystemJobRequestBus::BroadcastResult(
                    result, &AssetSystemJobRequestBus::Events::GetAssetJobsInfo, sourceAssetFullPath, true);
                JobInfoContainer& allJobs = result.GetValue();
                for (const JobInfo& job : allJobs)
                {
                    if (job.m_status == JobStatus::Queued || job.m_status == JobStatus::InProgress)
                    {
                        AZ_Printf("FooTest", "asset %s is being processed", sourceAssetFullPath.c_str());
                        allAssetProcessed = false;
                    }
                    else
                    {
                        AZ_Printf("FooTest", "asset %s is done", sourceAssetFullPath.c_str());
                    }
                }
            }

            if (allAssetProcessed)
            {
                AZ_Printf("ROS2EditorSystemComponent", "ROS2EditorSystemComponent::OnTick::In_Progress_Wait_For_AP::allAssetProcessed");
                m_importState = UrdfImportState::In_Progress_Create_Prefab;
            }
            return;
        }

        if (m_importState == UrdfImportState::In_Progress_Create_Prefab)
        {
            AZ_Printf("ROS2EditorSystemComponent", "ROS2EditorSystemComponent::OnTick::In_Progress_Create_Prefab");
            AZStd::string prefabName = AZStd::string(m_parsedUrdf->getName().c_str(), m_parsedUrdf->getName().size()) + ".prefab";

            const AZ::IO::Path prefabPathRelative(AZ::IO::Path("Assets") / "Importer" / prefabName);
            const AZ::IO::Path prefabPath(AZ::IO::Path(AZ::Utils::GetProjectPath()) / prefabPathRelative);
            AZStd::unique_ptr<URDFPrefabMaker> prefabMaker;
            prefabMaker = AZStd::make_unique<URDFPrefabMaker>(
                m_importedPath, m_parsedUrdf, prefabPath.String(), m_urdfAssetsMapping, m_useArticulation);

            auto prefabOutcome = prefabMaker->CreatePrefabFromURDF();

            if (!prefabOutcome.IsSuccess())
            {
                m_importState = UrdfImportState::Done;
            }
            m_importState = UrdfImportState::Failed;
            return;
        }

        if (m_importState == UrdfImportState::In_Progress)
        {
            AZ_Printf("ROS2EditorSystemComponent", "ROS2EditorSystemComponent::OnTick::Done");
            AZ::TickBus::Handler::BusDisconnect();
            return;
        }

        if (m_importState == UrdfImportState::Failed)
        {
            AZ_Printf("ROS2EditorSystemComponent", "ROS2EditorSystemComponent::OnTick::Failed");
            AZ::TickBus::Handler::BusDisconnect();
            return;
        }
    }

    bool ROS2RobotImporterEditorSystemComponent::IsDone()
    {
        return m_importState == UrdfImportState::Done;
    }

    bool ROS2RobotImporterEditorSystemComponent::IsSuccessful()
    {
        return m_importState == UrdfImportState::Done;
    }

} // namespace ROS2
