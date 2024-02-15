/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "Atom/RPI.Public/Pass/AttachmentReadback.h"
#include "AzCore/Component/EntityId.h"
#include "Camera/CameraSensorDescription.h"
#include "CameraPublishers.h"
#include <Atom/Feature/Utils/FrameCaptureBus.h>
#include <Atom/RPI.Public/Pass/AttachmentReadback.h>
#include <Atom/RPI.Reflect/System/RenderPipelineDescriptor.h>
#include <AzCore/std/containers/span.h>
#include <ROS2/ROS2GemUtilities.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace ROS2
{
    class CameraSensor
    {
    public:
        CameraSensor() = default;
        virtual ~CameraSensor() = default;

        //! Publish Image Message frame from rendering pipeline
        //! @param cameraPose - current camera pose from which the rendering should take place
        //! @param header - header with filled message information (frame, timestamp, seq)
        virtual void RequestMessagePublication(const AZ::Transform& cameraPose, const std_msgs::msg::Header& header) = 0;
    };

    //! Class to create camera sensor using Atom renderer
    //! It creates dedicated rendering pipeline for each camera
    class CameraSensorInternal
    {
    public:
        using AttachmentReadbackCallback = AZ::RPI::AttachmentReadback::CallbackFunction;

        //! Initializes rendering pipeline for the camera sensor.
        //! @param cameraSensorDescription - camera sensor description used to create camera pipeline.
        //! @param entityId - entityId for the owning sensor component.
        CameraSensorInternal(const CameraSensorDescription& cameraSensorDescription);

        //! Deinitializes rendering pipeline for the camera sensor
        virtual ~CameraSensorInternal();

        // void RequestFrameForChannel(const std_msgs::msg::Header& header, CameraSensorDescription::CameraChannelType channelType);
        void RequestColorFrame(const std_msgs::msg::Header& header);
        void RequestDepthFrame(const std_msgs::msg::Header& header);

        void RenderFrame(const AZ::Transform& cameraPose);

        //! Read and setup Atom Passes
        void SetupPasses(const AZ::RPI::RenderPipelineDescriptor& pipelineDesc);

    private:
        AttachmentReadbackCallback CreateAttachmentReadbackCallback(
            const std_msgs::msg::Header& header, CameraSensorDescription::CameraChannelType channelType);

    private:
        AZ::RPI::ViewPtr m_view;
        AZ::RPI::Scene* m_scene = nullptr;
        AZStd::string m_pipelineName;

        CameraSensorDescription m_cameraSensorDescription;
        CameraPublishers m_cameraPublishers;
        AZ::RPI::RenderPipelinePtr m_pipeline;
    };

    //! Implementation of camera sensors that runs pipeline which produces depth image
    class CameraDepthSensor : public CameraSensor
    {
    public:
        CameraDepthSensor(const CameraSensorDescription& cameraSensorDescription);

        void RequestMessagePublication(const AZ::Transform& cameraPose, const std_msgs::msg::Header& header) override;

    private:
        CameraSensorInternal m_cameraSensorInternal;
    };

    //! Implementation of camera sensors that runs pipeline which produces color image
    class CameraColorSensor : public CameraSensor
    {
    public:
        CameraColorSensor(const CameraSensorDescription& cameraSensorDescription);

        void RequestMessagePublication(const AZ::Transform& cameraPose, const std_msgs::msg::Header& header) override;

    private:
        CameraSensorInternal m_cameraSensorInternal;
    };

    //! Implementation of camera sensors that runs pipeline which produces color image and readbacks a depth image from pipeline
    class CameraRGBDSensor : public CameraSensor
    {
    public:
        CameraRGBDSensor(const CameraSensorDescription& cameraSensorDescription);

        void RequestMessagePublication(const AZ::Transform& cameraPose, const std_msgs::msg::Header& header) override;

    private:
        CameraSensorInternal m_cameraSensorInternal;
    };
} // namespace ROS2
