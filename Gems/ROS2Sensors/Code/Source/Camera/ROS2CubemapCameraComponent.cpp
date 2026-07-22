/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2CubemapCameraComponent.h"
#include "CameraUtilities.h"

#include <Atom/Feature/PostProcess/PostProcessFeatureProcessorInterface.h>
#include <Atom/Feature/Utils/FrameCaptureBus.h>
#include <Atom/RPI.Public/Pass/Specific/RenderToTexturePass.h>
#include <Atom/RPI.Public/RPISystemInterface.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/Math/Matrix3x4.h>
#include <AzCore/Math/Matrix4x4.h>
#include <AzCore/Math/Vector4.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <cmath>
#include <limits>

#include <ROS2/Clock/ROS2ClockRequestBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2NamesBus.h>

namespace ROS2Sensors
{
    namespace
    {
        // Camera basis vectors (right, up, back) for each cubemap face, matching the engine's own
        // EnvironmentCubeMapPass so the six faces tile a seamless cube. Standard cubemap face order:
        // +X, -X, +Y, -Y, +Z, -Z. These are expressed in the sensor's local frame and rotated into
        // world space by the entity orientation at capture time.
        AZStd::array<AZStd::array<AZ::Vector3, 3>, 6> MakeFaceBases()
        {
            return { { { { AZ::Vector3(0.0f, 1.0f, 0.0f), AZ::Vector3(-1.0f, 0.0f, 0.0f), AZ::Vector3(0.0f, 0.0f, 1.0f) } },
                       { { AZ::Vector3(0.0f, -1.0f, 0.0f), AZ::Vector3(1.0f, 0.0f, 0.0f), AZ::Vector3(0.0f, 0.0f, 1.0f) } },
                       { { AZ::Vector3(-1.0f, 0.0f, 0.0f), AZ::Vector3(0.0f, 0.0f, 1.0f), AZ::Vector3(0.0f, 1.0f, 0.0f) } },
                       { { AZ::Vector3(-1.0f, 0.0f, 0.0f), AZ::Vector3(0.0f, 0.0f, -1.0f), AZ::Vector3(0.0f, -1.0f, 0.0f) } },
                       { { AZ::Vector3(-1.0f, 0.0f, 0.0f), AZ::Vector3(0.0f, -1.0f, 0.0f), AZ::Vector3(0.0f, 0.0f, 1.0f) } },
                       { { AZ::Vector3(1.0f, 0.0f, 0.0f), AZ::Vector3(0.0f, 1.0f, 0.0f), AZ::Vector3(0.0f, 0.0f, 1.0f) } } } };
        }
    } // namespace

    void ROS2CubemapCameraComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2CubemapCameraComponent, AZ::Component>()
                ->Version(3)
                ->Field("Topic", &ROS2CubemapCameraComponent::m_topic)
                ->Field("FaceSize", &ROS2CubemapCameraComponent::m_faceSize)
                ->Field("EquirectWidth", &ROS2CubemapCameraComponent::m_equirectWidth);

            if (auto* editContext = serialize->GetEditContext())
            {
                editContext
                    ->Class<ROS2CubemapCameraComponent>(
                        "ROS2 Cubemap Camera", "Renders a cubemap around the entity and publishes it as a sensor_msgs/Image")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &ROS2CubemapCameraComponent::m_topic, "Topic", "Image topic name (namespaced)")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2CubemapCameraComponent::m_faceSize,
                        "Cube face size",
                        "Per-face render resolution in pixels (square)")
                    ->Attribute(AZ::Edit::Attributes::Min, 16)
                    ->Attribute(AZ::Edit::Attributes::Max, 2048)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2CubemapCameraComponent::m_equirectWidth,
                        "Equirect width",
                        "Output equirectangular image width in pixels (height is half)")
                    ->Attribute(AZ::Edit::Attributes::Min, 32)
                    ->Attribute(AZ::Edit::Attributes::Max, 8192);
            }
        }
    }

    void ROS2CubemapCameraComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2CubemapCameraComponent::Activate()
    {
        m_scene = AZ::RPI::RPISystemInterface::Get()->GetSceneByName(AZ::Name("Main"));
        AZ_Assert(m_scene, "ROS2CubemapCameraComponent requires a scene named 'Main'");

        const auto* frame = GetEntity()->FindComponent<ROS2::ROS2FrameComponent>();
        AZ_Assert(frame, "Entity has no ROS2FrameComponent");
        m_frameId = frame->GetNamespacedFrameID();

        AZStd::string fullTopic;
        ROS2::ROS2NamesRequestBus::BroadcastResult(
            fullTopic, &ROS2::ROS2NamesRequestBus::Events::GetNamespacedName, frame->GetNamespace(), m_topic);

        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        m_publisher = ros2Node->create_publisher<sensor_msgs::msg::Image>(fullTopic.data(), rclcpp::SensorDataQoS());

        SetupFacePipelines();
        BuildRemapTable();

        Camera::CameraNotificationBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void ROS2CubemapCameraComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        Camera::CameraNotificationBus::Handler::BusDisconnect();

        // Stop the self-sustaining loop: any in-flight readback callback will see this and not re-arm.
        {
            AZStd::lock_guard<AZStd::mutex> lock(m_mutex);
            m_shuttingDown = true;
        }

        if (m_scene)
        {
            if (auto* fp = m_scene->GetFeatureProcessor<AZ::Render::PostProcessFeatureProcessorInterface>())
            {
                for (auto& face : m_faces)
                {
                    if (face.m_view)
                    {
                        fp->RemoveViewAlias(face.m_view);
                    }
                }
            }
            for (auto& face : m_faces)
            {
                if (face.m_pipeline)
                {
                    m_scene->RemoveRenderPipeline(face.m_pipeline->GetId());
                }
            }
        }
        for (auto& face : m_faces)
        {
            face.m_pipeline.reset();
            face.m_view.reset();
        }
        m_publisher.reset();
        m_scene = nullptr;
    }

    void ROS2CubemapCameraComponent::SetupFacePipelines()
    {
        // Cube face: 90 deg FOV, square aspect. Near/far mirror the color camera defaults.
        const AZ::Matrix4x4 viewToClip = CameraUtils::MakeClipMatrix(m_faceSize, m_faceSize, 90.0f, 0.1f, 1000.0f);
        const auto multisampleState = AZ::RPI::RPISystemInterface::Get()->GetApplicationMultisampleState();

        for (int i = 0; i < NumFaces; ++i)
        {
            FacePipeline& face = m_faces[i];

            face.m_view = AZ::RPI::View::CreateView(AZ::Name("MainCamera"), AZ::RPI::View::UsageCamera);
            face.m_view->SetViewToClipMatrix(viewToClip);

            face.m_name = AZStd::string::format("CubemapFace%d_%s", i, GetEntityId().ToString().c_str());

            AZ::RPI::RenderPipelineDescriptor pipelineDesc;
            pipelineDesc.m_mainViewTagName = "MainCamera";
            pipelineDesc.m_name = face.m_name;
            pipelineDesc.m_rootPassTemplate = "PipelineRenderToTextureROSColor";
            pipelineDesc.m_renderSettings.m_multisampleState = multisampleState;

            face.m_pipeline = AZ::RPI::RenderPipeline::CreateRenderPipeline(pipelineDesc);
            face.m_pipeline->RemoveFromRenderTick();

            if (auto* rtt = azrtti_cast<AZ::RPI::RenderToTexturePass*>(face.m_pipeline->GetRootPass().get()))
            {
                rtt->ResizeOutput(m_faceSize, m_faceSize);
            }

            m_scene->AddRenderPipeline(face.m_pipeline);
            face.m_pipeline->SetDefaultView(face.m_view);

            face.m_passHierarchy = { face.m_name, "CopyToSwapChain" };
        }

        UpdateViewAliases();
    }

    void ROS2CubemapCameraComponent::UpdateViewAliases()
    {
        auto* fp = m_scene->GetFeatureProcessor<AZ::Render::PostProcessFeatureProcessorInterface>();
        if (!fp)
        {
            return;
        }
        const AZ::RPI::ViewPtr targetView = m_scene->GetDefaultRenderPipeline()->GetDefaultView();
        for (auto& face : m_faces)
        {
            if (face.m_view)
            {
                fp->SetViewAlias(face.m_view, targetView);
            }
        }
    }

    void ROS2CubemapCameraComponent::OnCameraRemoved([[maybe_unused]] const AZ::EntityId& cameraEntityId)
    {
        UpdateViewAliases();
    }

    void ROS2CubemapCameraComponent::OnActiveViewChanged([[maybe_unused]] const AZ::EntityId& cameraEntityId)
    {
        UpdateViewAliases();
    }

    void ROS2CubemapCameraComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        // Once running, the cycle is self-sustaining: the last face's readback callback publishes and
        // starts the next cycle. OnTick only kick-starts the first cycle and recovers if a cycle aborts.
        {
            AZStd::lock_guard<AZStd::mutex> lock(m_mutex);
            if (m_shuttingDown)
            {
                return;
            }
        }
        StartCaptureCycle();
    }

    void ROS2CubemapCameraComponent::StartCaptureCycle()
    {
        {
            AZStd::lock_guard<AZStd::mutex> lock(m_mutex);
            if (m_shuttingDown)
            {
                return;
            }
            m_capturing = true;
            m_facesReceived = 0;
        }

        const AZ::Transform worldTM = GetEntity()->GetTransform()->GetWorldTM();
        const AZ::Matrix3x3 worldRotation = AZ::Matrix3x3::CreateFromQuaternion(worldTM.GetRotation());
        const AZ::Vector3 worldPosition = worldTM.GetTranslation();
        const auto faceBases = MakeFaceBases();

        int failedToQueue = 0;
        for (int i = 0; i < NumFaces; ++i)
        {
            FacePipeline& face = m_faces[i];

            // Rotate the face basis (right, up, back) into world space and set it as the camera transform.
            AZ::Matrix3x4 cameraTransform;
            cameraTransform.SetBasisAndTranslation(
                worldRotation * faceBases[i][0], worldRotation * faceBases[i][1], worldRotation * faceBases[i][2], worldPosition);
            face.m_view->SetCameraTransform(cameraTransform);

            face.m_pipeline->AddToRenderTickOnce();
            if (!RequestFaceCapture(i))
            {
                ++failedToQueue;
                AZStd::lock_guard<AZStd::mutex> lock(m_mutex);
                m_faceData[i].clear();
            }
        }

        // Faces that failed to queue produce no async callback, so account for them here. In the normal
        // path the queued callbacks have not fired yet, so this only completes the cycle when *every*
        // face failed - in which case we abort and let OnTick retry next frame (no recursive re-arm).
        if (failedToQueue > 0)
        {
            AZStd::lock_guard<AZStd::mutex> lock(m_mutex);
            m_facesReceived += failedToQueue;
            if (m_facesReceived >= NumFaces)
            {
                m_capturing = false;
                AZ_Warning("ROS2CubemapCameraComponent", false, "Cubemap capture cycle aborted: no faces could be queued");
            }
        }
    }

    bool ROS2CubemapCameraComponent::RequestFaceCapture(int faceIndex)
    {
        auto callback = [this, faceIndex](const AZ::RPI::AttachmentReadback::ReadbackResult& result)
        {
            bool cycleComplete = false;
            {
                AZStd::lock_guard<AZStd::mutex> lock(m_mutex);
                if (result.m_state == AZ::RPI::AttachmentReadback::ReadbackState::Success && result.m_dataBuffer)
                {
                    m_faceData[faceIndex].assign(result.m_dataBuffer->begin(), result.m_dataBuffer->end());
                }
                else
                {
                    // Keep the cycle progressing even if a face failed; it will be zero-filled on assembly.
                    m_faceData[faceIndex].clear();
                }

                if (++m_facesReceived == NumFaces)
                {
                    cycleComplete = true;
                }
            }

            if (cycleComplete)
            {
                HandleCycleComplete();
            }
        };

        AZ::Render::FrameCaptureOutcome outcome;
        AZ::Render::FrameCaptureRequestBus::BroadcastResult(
            outcome,
            &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachmentWithCallback,
            callback,
            m_faces[faceIndex].m_passHierarchy,
            AZStd::string("Output"),
            AZ::RPI::PassAttachmentReadbackOption::Output);

        return outcome.IsSuccess();
    }

    void ROS2CubemapCameraComponent::HandleCycleComplete()
    {
        {
            AZStd::lock_guard<AZStd::mutex> lock(m_mutex);
            if (m_shuttingDown)
            {
                m_capturing = false;
                return;
            }
        }

        // Publish the completed cubemap, then immediately request the next one from here (the readback
        // callback) so we never wait for the next tick. Single-buffered, so the pipelines are free now.
        AssembleAndPublish();

    }

    void ROS2CubemapCameraComponent::BuildRemapTable()
    {
        const int width = m_equirectWidth;
        const int height = m_equirectWidth / 2;
        const int faceStep = m_faceSize * 4; // R8G8B8A8_UNORM bytes per row

        // Recreate the exact projection used to render the faces and, via temporary views with identity
        // translation, obtain each face's world-to-clip matrix in the sensor's local frame. Using Atom's
        // own matrices means the remap matches how pixels were actually rasterized (no convention guessing).
        const AZ::Matrix4x4 viewToClip = CameraUtils::MakeClipMatrix(m_faceSize, m_faceSize, 90.0f, 0.1f, 1000.0f);
        const auto faceBases = MakeFaceBases();
        AZStd::array<AZ::Matrix4x4, NumFaces> worldToClip;
        for (int i = 0; i < NumFaces; ++i)
        {
            auto view = AZ::RPI::View::CreateView(AZ::Name("CubemapRemapProbe"), AZ::RPI::View::UsageCamera);
            view->SetViewToClipMatrix(viewToClip);
            AZ::Matrix3x4 cameraTransform;
            cameraTransform.SetBasisAndTranslation(
                faceBases[i][0], faceBases[i][1], faceBases[i][2], AZ::Vector3::CreateZero());
            view->SetCameraTransform(cameraTransform);
            worldToClip[i] = view->GetWorldToClipMatrix();
        }

        m_remapFace.assign(static_cast<size_t>(width) * height, -1);
        m_remapOffset.assign(static_cast<size_t>(width) * height, 0);

        for (int y = 0; y < height; ++y)
        {
            // Latitude: +pi/2 at the top row (zenith) to -pi/2 at the bottom row.
            const float lat = AZ::Constants::HalfPi - (y + 0.5f) / height * AZ::Constants::Pi;
            for (int x = 0; x < width; ++x)
            {
                // Longitude: -pi..pi across the width, 0 (image centre) = sensor forward.
                const float lon = (x + 0.5f) / width * AZ::Constants::TwoPi - AZ::Constants::Pi;

                // Direction in the sensor's local (O3DE entity) frame: X right, Y forward, Z up.
                const AZ::Vector3 dir(
                    std::sin(lon) * std::cos(lat), std::cos(lon) * std::cos(lat), std::sin(lat));
                const AZ::Vector4 dir4(dir.GetX(), dir.GetY(), dir.GetZ(), 0.0f);

                int bestFace = -1;
                float bestCentering = std::numeric_limits<float>::max();
                float bestNdcX = 0.0f;
                float bestNdcY = 0.0f;
                for (int i = 0; i < NumFaces; ++i)
                {
                    const AZ::Vector4 clip = worldToClip[i] * dir4;
                    const float w = clip.GetW();
                    if (w <= 1e-6f)
                    {
                        continue; // Behind this face's view plane.
                    }
                    const float ndcX = clip.GetX() / w;
                    const float ndcY = clip.GetY() / w;
                    const float centering = AZ::GetMax(std::abs(ndcX), std::abs(ndcY));
                    if (centering <= 1.0f && centering < bestCentering)
                    {
                        bestCentering = centering;
                        bestFace = i;
                        bestNdcX = ndcX;
                        bestNdcY = ndcY;
                    }
                }

                if (bestFace < 0)
                {
                    continue;
                }

                int col = static_cast<int>((bestNdcX * 0.5f + 0.5f) * m_faceSize);
                int row = static_cast<int>((0.5f - bestNdcY * 0.5f) * m_faceSize);
                col = AZ::GetClamp(col, 0, m_faceSize - 1);
                row = AZ::GetClamp(row, 0, m_faceSize - 1);

                const size_t idx = static_cast<size_t>(y) * width + x;
                m_remapFace[idx] = bestFace;
                m_remapOffset[idx] = row * faceStep + col * 4;
            }
        }
    }

    void ROS2CubemapCameraComponent::AssembleAndPublish()
    {
        const uint32_t width = static_cast<uint32_t>(m_equirectWidth);
        const uint32_t height = static_cast<uint32_t>(m_equirectWidth / 2);
        const uint32_t step = width * 4; // rgba8

        sensor_msgs::msg::Image image;
        image.header.stamp = ROS2::ROS2ClockInterface::Get()->GetROSTimestamp();
        image.header.frame_id = m_frameId.c_str();
        image.width = width;
        image.height = height;
        image.encoding = "rgba8";
        image.step = step;
        image.is_bigendian = 0;
        image.data.assign(static_cast<size_t>(step) * height, 0);

        {
            AZStd::lock_guard<AZStd::mutex> lock(m_mutex);
            const size_t pixels = static_cast<size_t>(width) * height;
            for (size_t idx = 0; idx < pixels; ++idx)
            {
                const int face = m_remapFace[idx];
                if (face < 0)
                {
                    continue;
                }
                const auto& faceBuffer = m_faceData[face];
                const int offset = m_remapOffset[idx];
                if (offset >= 0 && static_cast<size_t>(offset) + 4 <= faceBuffer.size())
                {
                    memcpy(image.data.data() + idx * 4, faceBuffer.data() + offset, 4);
                }
            }
        }

        m_publisher->publish(image);
    }
} // namespace ROS2Sensors
