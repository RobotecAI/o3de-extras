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

#include <AzCore/Component/EntityBus.h>
#include <AzCore/Math/Spline.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/deque.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBodyEvents.h>
#include <AzFramework/Physics/Material/PhysicsMaterialAsset.h>
#include <AzFramework/Physics/PhysicsSystem.h>

namespace ROS2
{
    //! Component that simulates a conveyor belt using kinematic physics.
    //! The conveyor belt is simulated using a spline and number of kinematic rigid bodies.
    //! The kinematic rigid bodies have their kinematic targets set to interpolate along the spline.
    //! The component is updating kinematic targets every physic sub-step and creates and despawns rigid bodies as needed.
    class ConveyorBeltComponentKinematic
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public AZ::EntityBus::Handler
    {
        static constexpr float SegmentSeparation = 1.0f; //!< Separation between segments of the belt (in normalized units)

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
        // EntityBus::Handler overrides
        void OnEntityActivated(const AZ::EntityId& entityId) override;

        //! Obtains the start and end point of the simulated conveyor belt
        //! @param splinePtr the spline to obtain the start and end point from
        //! @return a pair of vectors, the first being the start point and the second being the end point
        AZStd::pair<AZ::Vector3, AZ::Vector3> GetStartAndEndPointOfBelt(AZ::ConstSplinePtr splinePtr);

        //! Obtain the length of the spline
        //! @param splinePtr the spline to obtain the length from
        float GetSplineLength(AZ::ConstSplinePtr splinePtr);

        //! Obtains location of the segment of the belt.
        //! @param handle the handle of the simulated body of the segment
        //! @return the location of the segment in world space
        AZ::Vector3 GetLocationOfSegment(const AzPhysics::SimulatedBodyHandle handle);

        //! Obtains the transform of of the pose on the spline at the given distance
        //! @param splinePtr the spline to obtain the transform from
        //! @param distanceNormalized the distance along the spline to obtain the transform from (normalized)
        //! @return the transform of the pose on the spline at the given distance
        AZ::Transform GetTransformFromSpline(AZ::ConstSplinePtr splinePtr, float distanceNormalized);

        //! Spawn rigid body at the given location
        //! @param splinePtr the spline to spawn the rigid body on
        //! @param location the location to spawn the rigid body at (normalized)
        //! @return a pair of the normalized location and the handle of the simulated body
        AZStd::pair<float, AzPhysics::SimulatedBodyHandle> CreateSegment(AZ::ConstSplinePtr splinePtr, float normalizedLocation);

        // AZ::TickBus::Handler overrides...
        void OnTick(float delta, AZ::ScriptTimePoint timePoint) override;

        //! Update texture offset of the conveyor belt
        //! @param deltaTime the time since the last update
        void MoveSegmentsGraphically(float deltaTime);

        //! Update location of segments in physics scene
        //! @param deltaTime the time since the last update
        void MoveSegmentsPhysically(float deltaTime);

        //! Despawn segments that are at the end of the spline
        void DespawnSegments();

        //! Spawn segments with given rate of spawning
        //! @param deltaTime the time since the last call of the function
        void SpawnSegments(float deltaTime);

        AzPhysics::SceneEvents::OnSceneSimulationFinishHandler m_sceneFinishSimHandler; //!< Handler called after every physics sub-step
        AZStd::deque<AZStd::pair<float, AzPhysics::SimulatedBodyHandle>> m_ConveyorSegments; //!< Cache of created segments
        float m_textureOffset = 0.0f; //!< Current offset of the texture during animation
        AZ::ConstSplinePtr m_splineConsPtr{ nullptr }; //!< Pointer to the spline
        float m_splineLength = -1.0f; //!< Non-normalized spline length
        AZ::Transform m_splineTransform; //!< Transform from spline's local frame to world frame
        AZ::Vector3 m_startPoint; //!< Start point of the belt
        AZ::Vector3 m_endPoint; //!< End point of the belt
        AzPhysics::SceneHandle m_sceneHandle; //!< Scene handle of the scene the belt is in
        AZStd::string m_graphicalMaterialSlot{ "Belt" }; //!< Name of the material slot to change UVs of

        float m_speed = 1.0f; //!< Speed of the conveyor belt.
        float m_beltWidth = 1.0f; //!< Width of the conveyor belt.
        float m_segmentLength = 1.0f; //!< Length of individual segments of the conveyor belt
        AZ::Data::Asset<Physics::MaterialAsset> m_materialAsset; //!< Material of individual segments of the conveyor belt
        AZ::EntityId m_ConveyorEntityId; //!< Conveyor belt entity (used for texture movement)
        float m_textureScale = 1.0f; //!< Scaling factor of the texture
        float m_deltaTimeFromLastSpawn = 0.0f; //!< Time since the last spawn
    };
} // namespace ROS2
