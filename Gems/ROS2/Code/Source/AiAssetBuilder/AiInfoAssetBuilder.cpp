/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "AiInfoAssetBuilder.h"

#include <AzCore/IO/FileIO.h>
#include <AzCore/IO/IOUtils.h>
#include <AzCore/Serialization/Json/JsonUtils.h>
#include <AzCore/Settings/SettingsRegistryVisitorUtils.h>
#include <AzToolsFramework/Entity/EntityUtilityComponent.h>
#include <AzToolsFramework/Prefab/PrefabLoaderInterface.h>
#include <AzToolsFramework/Prefab/PrefabLoaderScriptingBus.h>
#include <AzToolsFramework/Prefab/PrefabSystemComponentInterface.h>
#include <AzToolsFramework/Prefab/PrefabSystemScriptingBus.h>
#include <AzToolsFramework/Prefab/Procedural/ProceduralPrefabAsset.h>

#include <AssetBuilderSDK/AssetBuilderSDK.h>
#include <AssetBuilderSDK/SerializationDependencies.h>

#include "AiInfoAsset.h"
#include <AzFramework/Asset/XmlSchemaAsset.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableSystemComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzToolsFramework/Entity/PrefabEditorEntityOwnershipInterface.h>
#include <AzToolsFramework/Prefab/PrefabPublicInterface.h>
namespace ROS2
{

    constexpr const char* AiInfoAssetBuilderJobKey = "AiInfoAssetBuilder";

    AiInfoAssetBuilder::AiInfoAssetBuilder()
    {
        m_fingerprint = GetFingerprint();

        AssetBuilderSDK::AssetBuilderDesc aiInfoAssetBuilderDescriptor;

        aiInfoAssetBuilderDescriptor.m_name = AiInfoAssetBuilderJobKey;
        aiInfoAssetBuilderDescriptor.m_version = 1; // bump this to rebuild all sdf files
        aiInfoAssetBuilderDescriptor.m_busId = azrtti_typeid<AiInfoAssetBuilder>();
        aiInfoAssetBuilderDescriptor.m_patterns.push_back(
            AssetBuilderSDK::AssetBuilderPattern("*.prefab", AssetBuilderSDK::AssetBuilderPattern::PatternType::Wildcard));
        aiInfoAssetBuilderDescriptor.m_analysisFingerprint = m_fingerprint; // set the fingerprint to the global settings

        aiInfoAssetBuilderDescriptor.m_createJobFunction = [this](auto&& request, auto&& response)
        {
            return CreateJobs(request, response);
        };

        aiInfoAssetBuilderDescriptor.m_processJobFunction = [this](auto&& request, auto&& response)
        {
            return ProcessJob(request, response);
        };

        // Listen for asset builder notifications requesting jobs for any of the sdf source file types.
        BusConnect(aiInfoAssetBuilderDescriptor.m_busId);

        // Register this builder with the AssetBuilderSDK.
        AssetBuilderSDK::AssetBuilderBus::Broadcast(
            &AssetBuilderSDK::AssetBuilderBus::Handler::RegisterBuilderInformation, aiInfoAssetBuilderDescriptor);
    }

    AiInfoAssetBuilder::~AiInfoAssetBuilder()
    {
        // Stop listening for asset builder notifications.
        BusDisconnect();

        // The AssetBuilderSDK doesn't support deregistration, so there's nothing more to do here.
    }

    AZStd::string AiInfoAssetBuilder::GetFingerprint() const
    {
        return "todo";
    }

    void AiInfoAssetBuilder::CreateJobs(
        const AssetBuilderSDK::CreateJobsRequest& request, AssetBuilderSDK::CreateJobsResponse& response) const
    {
        const auto fullSourcePath = AZ::IO::Path(request.m_watchFolder) / AZ::IO::Path(request.m_sourceFile);

        // Create an output job for each platform
        for (const AssetBuilderSDK::PlatformInfo& platformInfo : request.m_enabledPlatforms)
        {
            AssetBuilderSDK::JobDescriptor jobDescriptor;
            jobDescriptor.m_critical = false;
            jobDescriptor.m_jobKey = "Ai Info Asset";
            jobDescriptor.SetPlatformIdentifier(platformInfo.m_identifier.c_str());

            jobDescriptor.m_additionalFingerprintInfo = m_fingerprint;

            response.m_createJobOutputs.push_back(jobDescriptor);
        }

        response.m_result = AssetBuilderSDK::CreateJobsResultCode::Success;
    }

    void AiInfoAssetBuilder::ProcessJob(
        const AssetBuilderSDK::ProcessJobRequest& request, AssetBuilderSDK::ProcessJobResponse& response) const
    {
        auto prefabEditorEntityOwnershipInterface = AZ::Interface<AzToolsFramework::PrefabEditorEntityOwnershipInterface>::Get();
        AZ_Assert(prefabEditorEntityOwnershipInterface, "Prefab editor entity ownership interface is not available.");

        auto result = prefabEditorEntityOwnershipInterface->InstantiatePrefab(AZ::IO::Path(request.m_sourceFile));
        if (!result)
        {
            AZ_TracePrintf(AssetBuilderSDK::ErrorWindow, "Error: Cannot load prefab");
            response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
            return;
        }
        AZ_Printf("AiInfoAssetBuilder", "Prefab loaded successfully");

        result->get().GetEntityIds(
            [](AZ::EntityId entityId)
            {
                AZ_Printf("AiInfoAssetBuilder", "EntityId: %s", entityId.ToString().c_str());
                return true;
            });

        auto tempAssetOutputPath = AZ::IO::Path(request.m_tempDirPath) / request.m_sourceFile;
        tempAssetOutputPath.ReplaceExtension("aiinfo");

        AZ::Data::Asset<AiInfoAsset> aiInfoAsset;
        aiInfoAsset.Create(AZ::Data::AssetId(AZ::Uuid::CreateRandom()));

        aiInfoAsset.Get()->m_name = request.m_sourceFile;
        aiInfoAsset.Get()->m_description = "This is a test description";

        if (auto assetHandler = AZ::Data::AssetManager::Instance().GetHandler(azrtti_typeid<AiInfoAsset>()))
        {
            AZ::IO::FileIOStream outStream(
                tempAssetOutputPath.String().c_str(), AZ::IO::OpenMode::ModeWrite | AZ::IO::OpenMode::ModeCreatePath);
            if (!outStream.IsOpen())
            {
                AZ_TracePrintf(
                    AssetBuilderSDK::ErrorWindow, "Error: Failed job %s because file cannot be created.\n", request.m_fullPath.c_str());
                response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
                return;
            }
            assetHandler->SaveAssetData(aiInfoAsset, &outStream);
        }

        // Save the asset to a file

        AssetBuilderSDK::JobProduct aiInfoJobProduct;
        aiInfoJobProduct.m_productFileName = tempAssetOutputPath.String();
        aiInfoJobProduct.m_productSubID = 0;
        aiInfoJobProduct.m_productAssetType = azrtti_typeid<AiInfoAsset>();

        response.m_outputProducts.push_back(AZStd::move(aiInfoJobProduct));
        response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Success;
    }

} // namespace ROS2
