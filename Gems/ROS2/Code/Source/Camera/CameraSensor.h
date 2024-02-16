/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "Camera/CameraPublishers.h"
#include <Atom/RPI.Public/Pass/AttachmentReadback.h>
#include <Atom/RPI.Reflect/System/RenderPipelineDescriptor.h>
#include <AzCore/Component/EntityId.h>
#include <ROS2/Camera/CameraSensorRequestBus.h>

namespace ROS2
{
    //! Class to create camera sensor using Atom renderer
    //! It creates dedicated rendering pipeline for each camera
    class CameraSensorInternal
    {
    public:
        using AttachmentReadbackCallback = AZ::RPI::AttachmentReadback::CallbackFunction;

        //! Initializes rendering pipeline for the camera sensor.
        //! @param cameraSensorDescription - camera sensor description used to create camera pipeline.
        //! @param entityId - entityId for the owning sensor component.
        explicit CameraSensorInternal(AZ::EntityId entityId, const AZ::RPI::RenderPipelineDescriptor& pipelineDesc);

        //! Deinitializes rendering pipeline for the camera sensor
        ~CameraSensorInternal();

        // void RequestFrameForChannel(const std_msgs::msg::Header& header, CameraSensorDescription::CameraChannelType channelType);
        void RequestColorFrame(const std_msgs::msg::Header& header);
        void RequestDepthFrame(const std_msgs::msg::Header& header);

        void RenderFrame(const AZ::Transform& cameraPose);

    private:
        AttachmentReadbackCallback CreateAttachmentReadbackCallback(
            const std_msgs::msg::Header& header, CameraPublishers::CameraChannelType channelType);

        AZ::RPI::ViewPtr m_view;
        AZ::RPI::Scene* m_scene = nullptr;
        AZStd::string m_pipelineName;

        CameraPublishers m_cameraPublishers;
        AZ::RPI::RenderPipelinePtr m_pipeline;
        AZ::EntityId m_entityId;
    };

    //! Implementation of camera sensors that runs pipeline which produces depth image
    class CameraDepthSensor : public CameraSensor
    {
    public:
        explicit CameraDepthSensor(AZ::EntityId entityId);

        void RequestMessagePublication(const AZ::Transform& cameraPose, const std_msgs::msg::Header& header) override;

    private:
        CameraSensorInternal m_cameraSensorInternal;
    };

    //! Implementation of camera sensors that runs pipeline which produces color image
    class CameraColorSensor : public CameraSensor
    {
    public:
        explicit CameraColorSensor(AZ::EntityId entityId);

        void RequestMessagePublication(const AZ::Transform& cameraPose, const std_msgs::msg::Header& header) override;

    private:
        CameraSensorInternal m_cameraSensorInternal;
    };

    //! Implementation of camera sensors that runs pipeline which produces color image and readbacks a depth image from pipeline
    class CameraRGBDSensor : public CameraSensor
    {
    public:
        explicit CameraRGBDSensor(AZ::EntityId entityId);

        void RequestMessagePublication(const AZ::Transform& cameraPose, const std_msgs::msg::Header& header) override;

    private:
        CameraSensorInternal m_cameraSensorInternal;
    };
} // namespace ROS2
