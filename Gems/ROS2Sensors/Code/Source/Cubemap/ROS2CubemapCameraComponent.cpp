/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2CubemapCameraComponent.h"
#include "EquirectangularConverter.h"
#include "FisheyeConverter.h"
#include "NaiveCubemapConverter.h"

#include <Camera/CameraUtilities.h>

#include <Atom/Feature/PostProcess/PostProcessFeatureProcessorInterface.h>
#include <Atom/Feature/Utils/FrameCaptureBus.h>
#include <Atom/RPI.Public/Pass/Specific/RenderToTexturePass.h>
#include <Atom/RPI.Public/RPISystemInterface.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Debug/Profiler.h>
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
                ->Version(5)
                ->Field("Topic", &ROS2CubemapCameraComponent::m_topic)
                ->Field("Projection", &ROS2CubemapCameraComponent::m_projection)
                ->Field("Interpolation", &ROS2CubemapCameraComponent::m_interpolation)
                ->Field("FaceSize", &ROS2CubemapCameraComponent::m_faceSize)
                ->Field("OutputWidth", &ROS2CubemapCameraComponent::m_outputWidth)
                ->Field("FisheyeFovDeg", &ROS2CubemapCameraComponent::m_fisheyeFovDeg);

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
                        AZ::Edit::UIHandlers::ComboBox,
                        &ROS2CubemapCameraComponent::m_projection,
                        "Projection",
                        "Projection model used to convert the cubemap into the published image")
                    ->EnumAttribute(CubemapProjection::Equirectangular, "Equirectangular")
                    ->EnumAttribute(CubemapProjection::Fisheye, "Fisheye")
                    ->EnumAttribute(CubemapProjection::Cubemap, "Cubemap (raw faces)")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &ROS2CubemapCameraComponent::m_interpolation,
                        "Interpolation",
                        "How the output samples the cube faces")
                    ->EnumAttribute(CubemapInterpolation::Nearest, "Nearest")
                    ->EnumAttribute(CubemapInterpolation::Bilinear, "Bilinear")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2CubemapCameraComponent::m_faceSize,
                        "Cube face size",
                        "Per-face render resolution in pixels (square)")
                    ->Attribute(AZ::Edit::Attributes::Min, 16)
                    ->Attribute(AZ::Edit::Attributes::Max, 2048)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2CubemapCameraComponent::m_outputWidth,
                        "Output width",
                        "Output image width in pixels (equirectangular height is half; fisheye is square)")
                    ->Attribute(AZ::Edit::Attributes::Min, 32)
                    ->Attribute(AZ::Edit::Attributes::Max, 8192)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2CubemapCameraComponent::m_fisheyeFovDeg,
                        "Fisheye FOV (deg)",
                        "Field of view for the fisheye projection in degrees")
                    ->Attribute(AZ::Edit::Attributes::Min, 30.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 360.0f);
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

        // Choose the projection model (what the published image looks like / which faces are rendered).
        switch (m_projection)
        {
        case CubemapProjection::Fisheye:
            m_converter = AZStd::make_unique<FisheyeConverter>(m_outputWidth, m_fisheyeFovDeg);
            break;
        case CubemapProjection::Cubemap:
            m_converter = AZStd::make_unique<NaiveCubemapConverter>();
            break;
        case CubemapProjection::Equirectangular:
        default:
            m_converter = AZStd::make_unique<EquirectangularConverter>(m_outputWidth);
            break;
        }
        m_converter->SetInterpolation(m_interpolation);
        m_converter->Initialize(m_faceSize, ComputeFaceWorldToClip());

        m_expectedFaces = 0;
        for (int i = 0; i < NumFaces; ++i)
        {
            if (m_converter->IsFaceRequired(i))
            {
                ++m_expectedFaces;
            }
        }

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
        AZ_PROFILE_SCOPE(ROS2Sensors, "ROS2CubemapCamera::StartCaptureCycle");
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
            // Only render faces the projection actually samples (e.g. a forward fisheye needs fewer).
            if (m_converter && !m_converter->IsFaceRequired(i))
            {
                continue;
            }

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
        // required face failed - in which case we abort and let OnTick retry next frame (no recursive re-arm).
        if (failedToQueue > 0)
        {
            AZStd::lock_guard<AZStd::mutex> lock(m_mutex);
            m_facesReceived += failedToQueue;
            if (m_facesReceived >= m_expectedFaces)
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
            AZ_PROFILE_SCOPE(ROS2Sensors, "ROS2CubemapCamera::callback");
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

                if (++m_facesReceived == m_expectedFaces)
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

    AZStd::array<AZ::Matrix4x4, ROS2CubemapCameraComponent::NumFaces> ROS2CubemapCameraComponent::ComputeFaceWorldToClip() const
    {
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
        return worldToClip;
    }

    void ROS2CubemapCameraComponent::AssembleAndPublish()
    {
        AZ_PROFILE_SCOPE(ROS2Sensors, "ROS2CubemapCamera::AssembleAndPublish");
        if (!m_converter)
        {
            return;
        }

        sensor_msgs::msg::Image image;
        image.header.stamp = ROS2::ROS2ClockInterface::Get()->GetROSTimestamp();
        image.header.frame_id = m_frameId.c_str();

        {
            AZStd::lock_guard<AZStd::mutex> lock(m_mutex);
            m_converter->Convert(m_faceData, image);
        }

        m_publisher->publish(image);
    }
} // namespace ROS2Sensors
