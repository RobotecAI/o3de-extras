/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <ROS2/Sensor/ROS2SensorComponent.h>

#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Frame/ROS2Transform.h>
#include <Atom/RPI.Public/Pass/AttachmentReadback.h>
#include <opencv2/core/mat.hpp>

namespace ROS2
{

    namespace CVHelpers
    {
        cv::Mat cvMatFromReadbackResult(const AZ::RPI::AttachmentReadback::ReadbackResult& result);
        std::vector<uint8_t> cvMatToVector(cv::Mat image);
    } // namespace CVHelpers


    class DepthLimitCameraSensorPostprocessor
    {
    public:
        std::vector<uint8_t> postProcess(const AZ::RPI::AttachmentReadback::ReadbackResult& result);
    };
} // namespace ROS2
