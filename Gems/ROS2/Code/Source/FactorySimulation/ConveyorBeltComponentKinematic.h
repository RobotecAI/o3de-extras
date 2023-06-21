/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AzCore/Math/Aabb.h"
#include "AzCore/std/containers/unordered_map.h"
#include "AzCore/std/containers/unordered_set.h"
#include "AzFramework/Physics/Common/PhysicsTypes.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TickBus.h>

#include <AzCore/Math/Spline.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/deque.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBodyEvents.h>
#include <AzFramework/Physics/PhysicsSystem.h>

namespace ROS2
{
    //! Component that simulates a conveyor belt using kinematic physics.
    //! The conveyor belt is simulated using a spline and number of kinematic rigid bodies.
    //! The kinimatic rigid bodies have their kinematic targets set to interpolate along the spline.
    //! The component is updating kinematic targets every physic sub-step and creates and despawns rigid bodies as needed.
    class ConveyorBeltComponentKinematic
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(ConveyorBeltComponentKinematic, "{B7F56411-01D4-48B0-8874-230C58A578BD}");
        ConveyorBeltComponentKinematic() = default;
        ~ConveyorBeltComponentKinematic() = default;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);

        // Component overrides
        void Activate() override;
        void Deactivate() override;

    private:
        //! Obtains the start and end point of the simulated conveyor belt
        //! @param splinePtr the spline to obtain the start and end point from
        //! @return a pair of vectors, the first being the start point and the second being the end point
        AZStd::pair<AZ::Vector3, AZ::Vector3> GetStartAndEndPointOfBelt(AZ::ConstSplinePtr splinePtr);

        //! Obtain the length of the spline
        //! @param splinePtr the spline to obtain the length from
        float GetSplineLength(AZ::ConstSplinePtr splinePtr);

        //! Obtains location of the segment of the belt.
        //! @param sceneHandle the scene handle of the scene the belt is in
        //! @param handle the handle of the simulated body of the segment
        //! @return the location of the segment in world space
        AZ::Vector3 GetLocationOfSegment(AzPhysics::SceneHandle sceneHandle, AzPhysics::SimulatedBodyHandle handle);

        //! Obtains the transform of of the pose on the spline at the given distance
        //! @param splinePtr the spline to obtain the transform from
        //! @param distanceNormalized the distance along the spline to obtain the transform from (normalized)
        //! @return the transform of the pose on the spline at the given distance
        AZ::Transform GetTransformFromSpline(AZ::ConstSplinePtr splinePtr, float distanceNormalized);

        //! Spawn rigid body at the given location
        //! @param splinePtr the spline to spawn the rigid body on
        //! @param physicsSystem the physics system to spawn the rigid body in
        //! @param sceneHandle the scene handle of the scene to spawn the rigid body in
        //! @param location the location to spawn the rigid body at (normalized)
        //! @return a pair of the normalized location and the handle of the simulated body
        AZStd::pair<float, AzPhysics::SimulatedBodyHandle> CreateSegment(
            AZ::ConstSplinePtr splinePtr,
            AzPhysics::SystemInterface* physicsSystem,
            AzPhysics::SceneHandle sceneHandle,
            float normalizedLocation);

        // AZ::TickBus::Handler overrides...
        void OnTick(float delta, AZ::ScriptTimePoint timePoint) override;

        //! Every physic update
        void PostPhysicsSubTick(float fixedDeltaTime);
        //! Every physic update - scene event handler
        AzPhysics::SceneEvents::OnSceneSimulationFinishHandler m_sceneFinishSimHandler;
        //! Speed of the conveyor belt
        float m_speed = 1.0f;
        //! Width of the conveyor belt
        float m_beltWidth = 1.0f;
        //! Length of individual segments of the conveyor belt
        float m_segmentLength = 1.0f;
        //! Cache of created segments
        AZStd::deque<AZStd::pair<float, AzPhysics::SimulatedBodyHandle>> m_ConveyorSegments;
        //! Heigh of belt
        static constexpr float m_segmentWidth = 0.1f;
        //! Pointer to the spline
        AZ::ConstSplinePtr m_splineConsPtr{ nullptr };
        //! Real spline length
        float m_splineLength{ -1.f };
        //! Transform from spline's local frame to world frame
        AZ::Transform m_splineTransform;
        //! Start and end point of the belt
        AZ::Vector3 m_startPoint;
        AZ::Vector3 m_endPoint;

        bool m_initilized{ false };
    };
} // namespace ROS2
