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
        cv::Mat cvMatFromReadbackResult(AZ::RPI::AttachmentReadback::ReadbackResult& result, int width, int height);
        std::vector<uint8_t> cvMatToVector(cv::Mat image);
    }

    namespace PostprocessorParams
    {
        const int disparityMethodBM = 0;
        const int disparityMethodSGBM = 1;
    }

    struct DisparityMapDescription
    {
        int ndisparities = 0;
        int SADWindowSize = 31;
        bool applyColormap = true;
        int disparityMethodBM = PostprocessorParams::disparityMethodBM;
    }; 

    //! 
    class CameraSensorPostprocessor 
    {
    public:
        CameraSensorPostprocessor();
        ~CameraSensorPostprocessor() {};

        void setDisparityMapParams(DisparityMapDescription params) {disparityMapParams = params;};

        std::vector<uint8_t>  findDisparityMap(AZ::RPI::AttachmentReadback::ReadbackResult& resultL, 
                AZ::RPI::AttachmentReadback::ReadbackResult& resultR, int width, int height);

    private:
        DisparityMapDescription disparityMapParams;

        cv::Mat findDisparityMapBM(cv::Mat &imgLeft, cv::Mat &imgRight);

        cv::Mat m_imageL;
        cv::Mat m_imageR;
        cv::Mat m_result;;

    };
} // namespace ROS2
