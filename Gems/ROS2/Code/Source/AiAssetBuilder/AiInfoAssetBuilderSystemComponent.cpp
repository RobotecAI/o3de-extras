/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "AiInfoAssetBuilderSystemComponent.h"
#include "AiInfoAsset.h"
#include "AiInfoAssetBuilder.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
namespace ROS2
{

    AiInfoAssetBuilderSystemComponent::AiInfoAssetBuilderSystemComponent() = default;
    AiInfoAssetBuilderSystemComponent::~AiInfoAssetBuilderSystemComponent() = default;

    void AiInfoAssetBuilderSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        AiInfoAsset::Reflect(context);
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<AiInfoAssetBuilderSystemComponent, AZ::Component>()->Version(0)->Attribute(
                AZ::Edit::Attributes::SystemComponentTags, AssetBuilderSDK::ComponentTags::AssetBuilder);
        }
    }

    void AiInfoAssetBuilderSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("AiInfoAssetBuilderService"));
    }

    void AiInfoAssetBuilderSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("AiInfoAssetBuilderService"));
    }

    void AiInfoAssetBuilderSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        // This doesn't require any services to exist before startup.
    }

    void AiInfoAssetBuilderSystemComponent::GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        // If the asset services exist at all, they should be started first so that the Sdf builder can register with them correctly.
        dependent.push_back(AZ_CRC_CE("AssetDatabaseService"));
        dependent.push_back(AZ_CRC_CE("AssetCatalogService"));
    }

    void AiInfoAssetBuilderSystemComponent::Activate()
    {
        auto* materialAsset =
            aznew AzFramework::GenericAssetHandler<AiInfoAsset>("PhysX Material", AiInfoAsset::AssetGroup, AiInfoAsset::FileExtension);
        ;
        materialAsset->Register();
        m_assetHandlers.emplace_back(materialAsset);

        m_aiInfoAssetBuilder = AZStd::make_unique<AiInfoAssetBuilder>();
    }

    void AiInfoAssetBuilderSystemComponent::Deactivate()
    {
        for (auto& assetHandler : m_assetHandlers)
        {
            if (auto aiAssetHandler = azrtti_cast<AzFramework::GenericAssetHandler<AiInfoAsset>*>(assetHandler.get());
                aiAssetHandler != nullptr)
            {
                aiAssetHandler->Unregister();
            }
        }
        m_assetHandlers.clear();
        m_aiInfoAssetBuilder.reset();
    }

} // namespace ROS2
