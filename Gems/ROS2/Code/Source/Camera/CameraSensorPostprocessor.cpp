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

namespace ROS2
{


    cv::Mat CVHelpers::cvMatFromReadbackResult(AZ::RPI::AttachmentReadback::ReadbackResult& result, int width, int height)
    {
        return cv::Mat(height, width, CV_8UC4, result.m_dataBuffer->data());
    }

    std::vector<uint8_t> CVHelpers::cvMatToVector(cv::Mat image)
    {
        return std::vector<uint8_t>(image.data, image.data + image.total()*image.channels());
    }


        //const AZ::RHI::ImageDescriptor& descriptor = result.m_imageDescriptor;
        //cv::Mat image = cv::Mat(descriptor.m_size.m_height, descriptor.m_size.m_width, CV_8UC4, result.m_dataBuffer->data());

        //message.data = std::vector<uint8_t>(outImg.data, outImg.data + outImg.total()*outImg.channels());

    CameraSensorPostprocessor::CameraSensorPostprocessor()
    {

    }


    std::vector<uint8_t>  CameraSensorPostprocessor::findDisparityMap(AZ::RPI::AttachmentReadback::ReadbackResult& resultL, 
                AZ::RPI::AttachmentReadback::ReadbackResult& resultR, int width, int height)
    {

        AZ_TracePrintf("Postprocess", "findDisparityMap");

        auto imageL = CVHelpers::cvMatFromReadbackResult(resultL, width, height);
        auto imageR = CVHelpers::cvMatFromReadbackResult(resultR, width, height);

        auto disparity = findDisparityMapBM(imageL, imageR);

        return CVHelpers::cvMatToVector(disparity);

    }

    cv::Mat CameraSensorPostprocessor::findDisparityMapBM(cv::Mat &imgLeft, cv::Mat &imgRight)
    {

        cv::imwrite("/home/pawel/tmp/L.png", imgLeft);
        cv::imwrite("/home/pawel/tmp/R.png", imgRight);

        cv::Mat grayR, grayL;        
        
        cv::cvtColor(imgRight, grayR, cv::COLOR_RGBA2GRAY);
        cv::cvtColor(imgLeft, grayL, cv::COLOR_RGBA2GRAY);

        cv::Mat imgDisparity16S = cv::Mat(imgRight.rows, imgRight.cols, CV_16S);
        cv::Mat imgDisparity8U = cv::Mat(imgRight.rows, imgRight.cols, CV_8UC1);
        
        cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(disparityMapParams.ndisparities, disparityMapParams.SADWindowSize);

        sbm->compute(grayL, grayR, imgDisparity16S);
        
        double minVal, maxVal;
        
        cv::minMaxLoc( imgDisparity16S, &minVal, &maxVal);
        
        imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));

        cv::Mat outImg;

        if(disparityMapParams.applyColormap)
        {
            cv::Mat heatmap;
            cv::applyColorMap(imgDisparity8U, heatmap, cv::COLORMAP_JET);
            //cv::applyColorMap(imgDisparity8U, heatmap, cv::COLORMAP_RAINBOW);

            cv::Mat outImg;
            cv::cvtColor(heatmap, outImg, cv::COLOR_BGR2RGBA);
        }
        else
        {
            cv::cvtColor(imgDisparity8U, outImg, cv::COLOR_GRAY2RGBA);
            
        }
        cv::imwrite("/home/pawel/tmp/D.png", outImg);

        return outImg;
    }

} // namespace ROS2
