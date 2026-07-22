/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <Atom/RPI.Public/Base.h>
#include <Atom/RPI.Public/Pass/AttachmentReadback.h>
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Public/Scene.h>
#include <Atom/RPI.Public/View.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/std/containers/array.h>
#include <AzCore/std/parallel/mutex.h>
#include <AzFramework/Components/CameraBus.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2Sensors
{
    //! Sensor component that renders a cubemap around the entity and publishes it as a single
    //! sensor_msgs/Image (six faces stacked vertically).
    //!
    //! Unlike a reflection-probe bake, this renders all six faces every capture cycle using six
    //! dedicated render-to-texture pipelines (the same pipeline the ROS2 color camera uses), and reads
    //! them back asynchronously. A new cycle starts as soon as the previous one is published, so the
    //! effective rate is limited by GPU cost and readback latency rather than an artificial per-face
    //! delay - it can run close to real time at modest face resolutions.
    class ROS2CubemapCameraComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public Camera::CameraNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2CubemapCameraComponent, ROS2CubemapCameraComponentTypeId, AZ::Component);

        ROS2CubemapCameraComponent() = default;
        ~ROS2CubemapCameraComponent() override = default;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;

    private:
        static constexpr int NumFaces = 6;

        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // Camera::CameraNotificationBus::Handler overrides (keep the view aliases in sync for post-processing)
        void OnCameraRemoved(const AZ::EntityId& cameraEntityId) override;
        void OnActiveViewChanged(const AZ::EntityId& cameraEntityId) override;

        //! One render-to-texture pipeline + view for a single cube face.
        struct FacePipeline
        {
            AZ::RPI::ViewPtr m_view;
            AZ::RPI::RenderPipelinePtr m_pipeline;
            AZStd::string m_name;
            AZStd::vector<AZStd::string> m_passHierarchy;
        };

        void SetupFacePipelines();
        void UpdateViewAliases();
        void StartCaptureCycle();
        //! Issue an async frame capture for one face. Returns true if the capture was queued (its
        //! callback will fire); false if it failed to start.
        bool RequestFaceCapture(int faceIndex);
        //! Called from the last face's readback callback: assemble + publish, then re-arm the next cycle.
        void HandleCycleComplete();
        void AssembleAndPublish();
        //! Precompute the equirectangular-pixel -> (cube face, face pixel) lookup table. Built once from
        //! the same projection used to render the faces, so it is pose-independent (sensor frame).
        void BuildRemapTable();

        AZStd::string m_topic{ "cubemap" };
        int m_faceSize = 512; //!< Per-face (cube face) resolution in pixels (square).
        int m_equirectWidth = 1024; //!< Output equirectangular width; height is half of this (2:1).

        AZStd::string m_frameId;
        AZ::RPI::Scene* m_scene = nullptr;
        AZStd::array<FacePipeline, NumFaces> m_faces;

        // Capture cycle state (m_faceData / counters are written from readback callbacks).
        AZStd::mutex m_mutex;
        AZStd::array<AZStd::vector<uint8_t>, NumFaces> m_faceData;
        int m_facesReceived = 0;
        bool m_capturing = false;
        bool m_shuttingDown = false; //!< Set on Deactivate so in-flight callbacks stop re-arming.

        // Equirectangular remap lookup table (size = m_equirectWidth * m_equirectWidth/2).
        // For each output pixel: the source cube face index (-1 if unmapped) and byte offset into that
        // face's readback buffer.
        AZStd::vector<int32_t> m_remapFace;
        AZStd::vector<int32_t> m_remapOffset;

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> m_publisher;
    };
} // namespace ROS2Sensors
