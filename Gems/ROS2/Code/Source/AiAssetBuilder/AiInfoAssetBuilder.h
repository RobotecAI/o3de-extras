/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AssetBuilderSDK/AssetBuilderBusses.h>
#include <AssetBuilderSDK/AssetBuilderSDK.h>

namespace ROS2
{
    [[maybe_unused]] constexpr const char* AiInfoAssetBuilderName = "AiInfoAssetBuilder";

    class AiInfoAssetBuilder : public AssetBuilderSDK::AssetBuilderCommandBus::Handler
    {
    public:
        AZ_RTTI(ROS2::AiInfoAssetBuilder, "{ac22dc2d-d6eb-437d-9b56-5fae82c01436}");

        AiInfoAssetBuilder();
        ~AiInfoAssetBuilder();

        // AssetBuilderSDK::AssetBuilderCommandBus overrides...
        void CreateJobs(const AssetBuilderSDK::CreateJobsRequest& request, AssetBuilderSDK::CreateJobsResponse& response) const;
        void ProcessJob(const AssetBuilderSDK::ProcessJobRequest& request, AssetBuilderSDK::ProcessJobResponse& response) const;
        void ShutDown() override
        {
        }

    private:
        AZStd::string GetFingerprint() const;
        AZStd::string m_fingerprint;
    };

} // namespace ROS2
