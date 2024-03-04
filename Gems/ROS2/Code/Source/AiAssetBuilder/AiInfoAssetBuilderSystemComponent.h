/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <AzFramework/Asset/GenericAssetHandler.h>
namespace AZ
{
    class ReflectContext;
}

namespace ROS2
{
    class AiInfoAssetBuilder;

    class AiInfoAssetBuilderSystemComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT(AiInfoAssetBuilderSystemComponent, "{91a2f55e-461a-4707-8a7d-18e7d53a08ec}");
        static void Reflect(AZ::ReflectContext* context);

        AiInfoAssetBuilderSystemComponent();
        ~AiInfoAssetBuilderSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // Component overrides ...
        void Activate() override;
        void Deactivate() override;

        // Asset builder instance
        AZStd::unique_ptr<AiInfoAssetBuilder> m_aiInfoAssetBuilder;

        // Assets related data
        AZStd::vector<AZStd::unique_ptr<AZ::Data::AssetHandler>> m_assetHandlers;
    };
} // namespace ROS2
