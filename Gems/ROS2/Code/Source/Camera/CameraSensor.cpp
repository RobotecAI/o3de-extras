/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "CameraSensor.h"
#include "AzCore/Component/Entity.h"
#include "AzCore/Debug/Trace.h"
#include "Camera/CameraPublishers.h"
#include "CameraUtilities.h"
#include <Atom/RPI.Public/Base.h>
#include <Atom/RPI.Public/FeatureProcessorFactory.h>
#include <Atom/RPI.Public/Pass/PassFactory.h>
#include <Atom/RPI.Public/Pass/PassSystemInterface.h>
#include <Atom/RPI.Public/Pass/Specific/RenderToTexturePass.h>
#include <Atom/RPI.Public/RPISystemInterface.h>
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Public/Scene.h>
#include <AzCore/Math/MatrixUtils.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Scene/SceneSystemInterface.h>
#include <PostProcess/PostProcessFeatureProcessor.h>
#include <ROS2/Camera/CameraCalibrationRequestBus.h>
#include <ROS2/Camera/CameraPostProcessingRequestBus.h>
#include <sensor_msgs/distortion_models.hpp>

namespace ROS2
{
    namespace Internal
    {
        /// @FormatMappings - contains the mapping from RHI to ROS image encodings. List of supported
        /// ROS image encodings lives in `sensor_msgs/image_encodings.hpp`
        /// We are not including `image_encodings.hpp` since it uses exceptions.
        const AZStd::unordered_map<AZ::RHI::Format, const char*> FormatMappings{
            { AZ::RHI::Format::R8G8B8A8_UNORM, "rgba8" },
            { AZ::RHI::Format::R32_FLOAT, "32FC1" },
        };

        /// @BitDepth - contains the mapping from RHI to size used in `step` size computation.
        /// It is some equivalent to `bitDepth()` function from `sensor_msgs/image_encodings.hpp`
        const AZStd::unordered_map<AZ::RHI::Format, int> BitDepth{
            { AZ::RHI::Format::R8G8B8A8_UNORM, 4 * sizeof(uint8_t) },
            { AZ::RHI::Format::R32_FLOAT, sizeof(float) },
        };

        //! Create a CameraImage message from the read-back result and a header.
        auto CreateImageMessageFromReadBackResult(
            const AZ::EntityId& entityId, const AZ::RPI::AttachmentReadback::ReadbackResult& result, const std_msgs::msg::Header& header)
            -> sensor_msgs::msg::Image
        {
            const AZ::RHI::ImageDescriptor& descriptor = result.m_imageDescriptor;
            const auto format = descriptor.m_format;
            AZ_Assert(Internal::FormatMappings.contains(format), "Unknown format in result %u", static_cast<uint32_t>(format));
            sensor_msgs::msg::Image imageMessage;
            imageMessage.encoding = Internal::FormatMappings.at(format);
            imageMessage.width = descriptor.m_size.m_width;
            imageMessage.height = descriptor.m_size.m_height;
            imageMessage.step = imageMessage.width * Internal::BitDepth.at(format);
            imageMessage.data =
                std::vector<uint8_t>(result.m_dataBuffer->data(), result.m_dataBuffer->data() + result.m_dataBuffer->size());
            imageMessage.header = header;
            CameraPostProcessingRequestBus::Event(entityId, &CameraPostProcessingRequests::ApplyPostProcessing, imageMessage);
            return imageMessage;
        }

        //! Prepare a CameraInfo message from sensor description and a header.
        auto CreateCameraInfoMessage(const AZ::EntityId& entityId, const std_msgs::msg::Header& header) -> sensor_msgs::msg::CameraInfo
        {
            sensor_msgs::msg::CameraInfo cameraInfo;
            CameraCalibrationRequestBus::EventResult(cameraInfo.width, entityId, &CameraCalibrationRequest::GetWidth);
            CameraCalibrationRequestBus::EventResult(cameraInfo.height, entityId, &CameraCalibrationRequest::GetHeight);
            AZ::Matrix3x3 cameraIntrinsics;
            CameraCalibrationRequestBus::EventResult(cameraIntrinsics, entityId, &CameraCalibrationRequest::GetCameraMatrix);
            cameraInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

            [[maybe_unused]] constexpr size_t expectedMatrixSize = 9;
            AZ_Assert(cameraInfo.k.size() == expectedMatrixSize, "camera matrix should have %d elements", expectedMatrixSize);
            cameraInfo.k = { cameraIntrinsics.GetElement(0, 0),
                             0,
                             cameraIntrinsics.GetElement(0, 2),
                             0,
                             cameraIntrinsics.GetElement(1, 1),
                             cameraIntrinsics.GetElement(1, 2),
                             0,
                             0,
                             1 };
            cameraInfo.p = { cameraInfo.k[0], cameraInfo.k[1], cameraInfo.k[2], 0, cameraInfo.k[3], cameraInfo.k[4], cameraInfo.k[5], 0,
                             cameraInfo.k[6], cameraInfo.k[7], cameraInfo.k[8], 0 };
            cameraInfo.header = header;
            return cameraInfo;
        }

        auto GetCameraName(AZ::EntityId entityId) -> AZStd::string
        {
            AZStd::string entityName = AZ::Interface<AZ::ComponentApplicationRequests>::Get()->GetEntityName(entityId);
            entityName.replace(entityName.begin(), entityName.end(), '/', '_');
            entityName.replace(entityName.begin(), entityName.end(), ' ', '_');

            AZStd::string cameraName = AZStd::string::format("camera_%s[%s]", entityName.c_str(), entityId.ToString().c_str());
            return cameraName;
        }

    } // namespace Internal

    CameraSensorInternal::CameraSensorInternal(AZ::EntityId entityId)
        : m_cameraPublishers(entityId)
        , m_entityId(entityId)
    {
    }

    void CameraSensorInternal::SetupPasses(const AZ::RPI::RenderPipelineDescriptor& pipelineDesc)
    {
        const AZ::Name viewName = AZ::Name("MainCamera");

        CameraSensorConfiguration configuration;
        CameraCalibrationRequestBus::EventResult(configuration, m_entityId, &CameraCalibrationRequest::GetCameraSensorConfiguration);

        auto viewToClipMatrix = CameraUtils::MakeClipMatrix(
            configuration.m_width,
            configuration.m_height,
            configuration.m_verticalFieldOfViewDeg,
            configuration.m_nearClipDistance,
            configuration.m_farClipDistance);

        m_view = AZ::RPI::View::CreateView(viewName, AZ::RPI::View::UsageCamera);
        m_view->SetViewToClipMatrix(viewToClipMatrix);
        m_scene = AZ::RPI::RPISystemInterface::Get()->GetSceneByName(AZ::Name("Main"));

        m_pipelineName = pipelineDesc.m_name;
        m_pipeline = AZ::RPI::RenderPipeline::CreateRenderPipeline(pipelineDesc);
        m_pipeline->RemoveFromRenderTick();

        if (auto* renderToTexturePass = azrtti_cast<AZ::RPI::RenderToTexturePass*>(m_pipeline->GetRootPass().get()))
        {
            renderToTexturePass->ResizeOutput(configuration.m_width, configuration.m_height);
        }

        m_scene->AddRenderPipeline(m_pipeline);
        m_pipeline->SetDefaultView(m_view);
        const AZ::RPI::ViewPtr targetView = m_scene->GetDefaultRenderPipeline()->GetDefaultView();
        if (auto* fp = m_scene->GetFeatureProcessor<AZ::Render::PostProcessFeatureProcessor>())
        {
            fp->SetViewAlias(m_view, targetView);
        }
    }

    CameraSensorInternal::~CameraSensorInternal()
    {
        if (m_scene != nullptr)
        {
            if (auto* fp = m_scene->GetFeatureProcessor<AZ::Render::PostProcessFeatureProcessor>())
            {
                fp->RemoveViewAlias(m_view);
            }
            m_scene->RemoveRenderPipeline(m_pipeline->GetId());
            m_scene = nullptr;
        }
        m_pipeline.reset();
        m_view.reset();
    }

    void CameraSensorInternal::RequestColorFrame(const std_msgs::msg::Header& header)
    {
        auto callback = CreateAttachmentReadbackCallback(header, CameraPublishers::CameraChannelType::RGB);
        AZStd::vector<AZStd::string> passHierarchy = { m_pipelineName };
        AZStd::string slotName = "Output";

        AZ::Render::FrameCaptureOutcome captureOutcome;
        AZ::Render::FrameCaptureRequestBus::BroadcastResult(
            captureOutcome,
            &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachmentWithCallback,
            callback,
            passHierarchy,
            slotName,
            AZ::RPI::PassAttachmentReadbackOption::Output);

        AZ_Error(
            "CameraSensor",
            captureOutcome.IsSuccess(),
            "Color Frame capture initialization failed. %s",
            captureOutcome.GetError().m_errorMessage.c_str());
    }

    void CameraSensorInternal::RequestDepthFrame(const std_msgs::msg::Header& header)
    {
        auto callback = CreateAttachmentReadbackCallback(header, CameraPublishers::CameraChannelType::DEPTH);
        AZStd::vector<AZStd::string> passHierarchy = { m_pipelineName, "DepthPrePass" };
        AZStd::string slotName = "DepthLinear";

        AZ::Render::FrameCaptureOutcome captureOutcome;
        AZ::Render::FrameCaptureRequestBus::BroadcastResult(
            captureOutcome,
            &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachmentWithCallback,
            callback,
            passHierarchy,
            slotName,
            AZ::RPI::PassAttachmentReadbackOption::Output);

        AZ_Error(
            "CameraSensor",
            captureOutcome.IsSuccess(),
            "Depth Frame capture initialization failed. %s",
            captureOutcome.GetError().m_errorMessage.c_str());
    }

    CameraSensorInternal::AttachmentReadbackCallback CameraSensorInternal::CreateAttachmentReadbackCallback(
        const std_msgs::msg::Header& header, CameraPublishers::CameraChannelType channelType)
    {
        auto imagePublisher = m_cameraPublishers.GetImagePublisher(channelType);
        auto infoPublisher = m_cameraPublishers.GetInfoPublisher(channelType);
        AZ_Assert(imagePublisher && infoPublisher, "Missing publisher for the Camera sensor");

        auto infoMessage = Internal::CreateCameraInfoMessage(m_entityId, header);
        return [header, imagePublisher, infoPublisher, infoMessage, entityId = m_entityId](
                   const AZ::RPI::AttachmentReadback::ReadbackResult& result)
        {
            if (result.m_state != AZ::RPI::AttachmentReadback::ReadbackState::Success)
            {
                AZ_Error("CameraSensor", false, "Readback failed with state %d", static_cast<int>(result.m_state));
                return;
            }

            auto imageMessage = Internal::CreateImageMessageFromReadBackResult(entityId, result, header);
            imagePublisher->publish(imageMessage);
            infoPublisher->publish(infoMessage);
        };
    }

    void CameraSensorInternal::RenderFrame(const AZ::Transform& cameraPose)
    {
        static const AZ::Transform AtomToRos{ AZ::Transform::CreateFromQuaternion(
            AZ::Quaternion::CreateFromMatrix3x3(AZ::Matrix3x3::CreateFromRows({ 1, 0, 0 }, { 0, -1, 0 }, { 0, 0, -1 }))) };

        const AZ::Transform inverse = (cameraPose * AtomToRos).GetInverse();
        m_view->SetWorldToViewMatrix(AZ::Matrix4x4::CreateFromQuaternionAndTranslation(inverse.GetRotation(), inverse.GetTranslation()));
        m_pipeline->AddToRenderTickOnce();
    }

    CameraDepthSensor::CameraDepthSensor(AZ::EntityId entityId)
        : m_cameraSensorInternal(entityId)
    {
        auto cameraName = Internal::GetCameraName(entityId);

        AZ::RPI::RenderPipelineDescriptor pipelineDesc;
        pipelineDesc.m_rootPassTemplate = "PipelineRenderToTextureROSDepth";
        pipelineDesc.m_mainViewTagName = "MainCamera";
        pipelineDesc.m_name = AZStd::string::format("ROSDepthPipeline_%s", cameraName.c_str());
        pipelineDesc.m_renderSettings.m_multisampleState = AZ::RPI::RPISystemInterface::Get()->GetApplicationMultisampleState();
        m_cameraSensorInternal.SetupPasses(pipelineDesc);
    }

    void CameraDepthSensor::RequestMessagePublication(const AZ::Transform& cameraPose, const std_msgs::msg::Header& header)
    {
        m_cameraSensorInternal.RenderFrame(cameraPose);
        m_cameraSensorInternal.RequestDepthFrame(header);
    }

    CameraColorSensor::CameraColorSensor(AZ::EntityId entityId)
        : m_cameraSensorInternal(entityId)
    {
        auto cameraName = Internal::GetCameraName(entityId);

        AZ::RPI::RenderPipelineDescriptor pipelineDesc;
        pipelineDesc.m_rootPassTemplate = "PipelineRenderToTextureROSColor";
        pipelineDesc.m_mainViewTagName = "MainCamera";
        pipelineDesc.m_name = AZStd::string::format("ROSColorPipeline_%s", cameraName.c_str());
        pipelineDesc.m_renderSettings.m_multisampleState = AZ::RPI::RPISystemInterface::Get()->GetApplicationMultisampleState();
        m_cameraSensorInternal.SetupPasses(pipelineDesc);
    }

    void CameraColorSensor::RequestMessagePublication(const AZ::Transform& cameraPose, const std_msgs::msg::Header& header)
    {
        m_cameraSensorInternal.RenderFrame(cameraPose);
        m_cameraSensorInternal.RequestColorFrame(header);
    }

    CameraRGBDSensor::CameraRGBDSensor(AZ::EntityId entityId)
        : m_cameraSensorInternal(entityId)
    {
        auto cameraName = Internal::GetCameraName(entityId);

        AZ::RPI::RenderPipelineDescriptor pipelineDesc;
        pipelineDesc.m_rootPassTemplate = "PipelineRenderToTextureROSColor";
        pipelineDesc.m_mainViewTagName = "MainCamera";
        pipelineDesc.m_name = AZStd::string::format("ROSRGBDPipeline_%s", cameraName.c_str());
        pipelineDesc.m_renderSettings.m_multisampleState = AZ::RPI::RPISystemInterface::Get()->GetApplicationMultisampleState();
        m_cameraSensorInternal.SetupPasses(pipelineDesc);
    }

    void CameraRGBDSensor::RequestMessagePublication(const AZ::Transform& cameraPose, const std_msgs::msg::Header& header)
    {
        m_cameraSensorInternal.RenderFrame(cameraPose);
        m_cameraSensorInternal.RequestColorFrame(header);
        m_cameraSensorInternal.RequestDepthFrame(header);
    }
} // namespace ROS2
