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

    std::vector<uint8_t> CVHelpers::cvMatToVector(const cv::Mat& image)
    {
        return std::vector<uint8_t>(image.data, image.data + image.rows * image.step);
    }

    sensor_msgs::msg::Image DepthLimitCameraSensorPostprocessor::postProcess(const AZ::RPI::AttachmentReadback::ReadbackResult& result)
    {
        static std::random_device rd{};
        std::mt19937 gen{rd()};

        // values near the mean are the most likely
        // standard deviation affects the dispersion of generated values from the mean
        std::uniform_int_distribution<uint16_t> d0{0, 300};
        std::uniform_int_distribution<uint16_t> d1{150,250};


        const float maxDepth= 5.f;
        const float minDepth= 0.3f;

        auto clamp = [maxDepth,minDepth](float f){
            return std::max(std::min(f,maxDepth),minDepth);
        };

        AZ_Assert(result.m_imageDescriptor.m_format == AZ::RHI::Format::R32_FLOAT, "DepthLimitCameraSensorPostprocessor works only with  R32_FLOAT");
        AZ_TracePrintf("CameraSensorPostprocessor", "postProcess");
        cv::Mat image = CVHelpers::cvMatFromReadbackResult(result);
        cv::Mat image_uc16(image.rows,image.cols,CV_16UC1);
        for( int j = 0; j < image.cols; ++j) {
            for( int i = 0; i < image.rows; ++i) {
                const float depth = image.at<float>(i,j);
                const float newdepth = clamp(depth);
                image_uc16.at<unsigned short>(i,j) = 1000*newdepth + d0(gen) + d1(gen);
            }
        }
        sensor_msgs::msg::Image msg;
        msg.data = CVHelpers::cvMatToVector(image_uc16);
        msg.encoding = "16UC1";
        msg.height = image_uc16.rows;
        msg.width = image_uc16.cols;
        msg.step = image_uc16.step;
        AZ_Printf("PostProcess", "Image size check height : %d, width %d, step %d, size: %d", msg.height,msg.width, msg.step, msg.data.size());
        AZ_Assert(msg.height*msg.step == msg.data.size(),"Image size check height : %d, width %d, step %d, size: %d", msg.height,msg.width, msg.step, msg.data.size());
        return msg;
    }

} // namespace ROS2
