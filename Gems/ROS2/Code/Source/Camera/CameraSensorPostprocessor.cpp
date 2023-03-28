/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "CameraSensorPostprocessor.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <random>

namespace ROS2
{

    namespace {
        /// @FormatMappings - contains the mapping from RHI to OpenCV image encodings.
        AZStd::unordered_map<AZ::RHI::Format, int> FormatMappingsOpenCV{
            { AZ::RHI::Format::R8G8B8A8_UNORM, CV_8UC4},
            { AZ::RHI::Format::R32_FLOAT, CV_32FC1 },
        };
    }

    cv::Mat CVHelpers::cvMatFromReadbackResult(const AZ::RPI::AttachmentReadback::ReadbackResult& result)
    {
        const auto format = FormatMappingsOpenCV.find(result.m_imageDescriptor.m_format);
        AZ_Error("CameraSensorPostprocessor", format != FormatMappingsOpenCV.end(), "Unknown image format");
        const int height = result.m_imageDescriptor.m_size.m_height;
        const int width = result.m_imageDescriptor.m_size.m_width;
        const int type = format->second;
        cv::Mat outMat = cv::Mat(height,width,type,result.m_dataBuffer->data());
        return outMat;
    }

    std::vector<uint8_t> CVHelpers::cvMatToVector(cv::Mat image)
    {
        return std::vector<uint8_t>(image.data, image.data + image.total() * image.elemSize());
    }

    std::vector<uint8_t> DepthLimitCameraSensorPostprocessor::postProcess(const AZ::RPI::AttachmentReadback::ReadbackResult& result)
    {
        static std::random_device rd{};
        std::mt19937 gen{rd()};

        // values near the mean are the most likely
        // standard deviation affects the dispersion of generated values from the mean
        std::normal_distribution<float> d{0, 0.1};

        const float maxDepth= 5.f;
        const float minDepth= 0.3f;

        auto clamp = [maxDepth,minDepth](float f){
            return std::max(std::min(f,maxDepth),minDepth);
        };

        AZ_Assert(result.m_imageDescriptor.m_format == AZ::RHI::Format::R32_FLOAT, "DepthLimitCameraSensorPostprocessor works only with  R32_FLOAT");
        AZ_TracePrintf("CameraSensorPostprocessor", "postProcess");
        cv::Mat image = CVHelpers::cvMatFromReadbackResult(result);
        for( int j = 0; j < image.cols; ++j) {
            for( int i = 0; i < image.rows; ++i) {
                float &depth = image.at<float>(i,j) ;
                depth = clamp(clamp(depth)*(1.f+d(gen)));
            }
        }
        return CVHelpers::cvMatToVector(image);
    }

} // namespace ROS2
