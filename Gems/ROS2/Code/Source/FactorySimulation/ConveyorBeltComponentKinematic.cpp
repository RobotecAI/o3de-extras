/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "ConveyorBeltComponentKinematic.h"
#include <AtomLyIntegration/CommonFeatures/Material/MaterialComponentBus.h>
#include <AzCore/Asset/AssetSerializer.h>
#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>
#include <AzFramework/Physics/Components/SimulatedBodyComponentBus.h>
#include <AzFramework/Physics/Material/PhysicsMaterialManager.h>
#include <AzFramework/Physics/Material/PhysicsMaterialSlots.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <LmbrCentral/Shape/SplineComponentBus.h>
#include <Source/RigidBodyComponent.h>

namespace ROS2
{
    static AZ::Data::AssetId GetDefaultPhysicsMaterialAssetId()
    {
        // Used for Edit Context.
        // When the physics material asset property doesn't have an asset assigned it
        // will show "(default)" to indicate that the default material will be used.
        if (auto* materialManager = AZ::Interface<Physics::MaterialManager>::Get())
        {
            if (AZStd::shared_ptr<Physics::Material> defaultMaterial = materialManager->GetDefaultMaterial())
            {
                return defaultMaterial->GetMaterialAsset().GetId();
            }
        }
        return {};
    }

    void ConveyorBeltComponentKinematic::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ConveyorBeltComponentKinematic>()
                ->Version(1)
                ->Field("BeltEntityId", &ConveyorBeltComponentKinematic::m_ConveyorEntityId)
                ->Field("Speed", &ConveyorBeltComponentKinematic::m_speed)
                ->Field("BeltWidth", &ConveyorBeltComponentKinematic::m_beltWidth)
                ->Field("SegmentLength", &ConveyorBeltComponentKinematic::m_segmentLength)
                ->Field("TextureScale", &ConveyorBeltComponentKinematic::m_textureScale)
                ->Field("MaterialAsset", &ConveyorBeltComponentKinematic::m_materialAsset)
                ->Field("GraphicalMaterialSlot", &ConveyorBeltComponentKinematic::m_graphicalMaterialSlot);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ConveyorBeltComponentKinematic>("Conveyor Belt Component Kinematic", "Conveyor Belt Component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &ConveyorBeltComponentKinematic::m_ConveyorEntityId,
                        "Conveyor Belt Entity",
                        "Entity of the conveyor belt")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &ConveyorBeltComponentKinematic::m_speed, "Speed", "Speed of the conveyor belt")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ConveyorBeltComponentKinematic::m_beltWidth, "Width", "Belt width")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ConveyorBeltComponentKinematic::m_segmentLength,
                        "Segment Length",
                        "Length of simulated segments. Short segments might affect performance"
                        "physics engine.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ConveyorBeltComponentKinematic::m_textureScale,
                        "Texture scale",
                        "Scale of the texture on conveyor belt.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ConveyorBeltComponentKinematic::m_materialAsset,
                        "Physical Material",
                        "Physical Material asset of the conveyor belt")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ConveyorBeltComponentKinematic::m_graphicalMaterialSlot,
                        "Graphical Material slot ",
                        "The graphical material slot name to have its UV coordinates animated.")
                    ->Attribute(AZ::Edit::Attributes::DefaultAsset, &GetDefaultPhysicsMaterialAssetId)
                    ->Attribute(AZ_CRC_CE("EditButton"), "")
                    ->Attribute(AZ_CRC_CE("EditDescription"), "Open in Asset Editor")
                    ->Attribute(AZ_CRC_CE("DisableEditButtonWhenNoAssetSelected"), true);
            }
        }
    }

    void ConveyorBeltComponentKinematic::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("SplineService"));
    }

    void ConveyorBeltComponentKinematic::Activate()
    {
        AZ::EntityBus::Handler::BusConnect(m_ConveyorEntityId);
        AZ::EntityBus::Handler::BusConnect(m_entity->GetId());
    }

    void ConveyorBeltComponentKinematic::Deactivate()
    {
        m_sceneFinishSimHandler.Disconnect();
        AZ::EntityBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
    }

    AZ::Vector3 ConveyorBeltComponentKinematic::GetLocationOfSegment(const AzPhysics::SimulatedBodyHandle handle)
    {
        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "No scene interface");
        auto* body = azdynamic_cast<AzPhysics::RigidBody*>(sceneInterface->GetSimulatedBodyFromHandle(m_sceneHandle, handle));
        AZ_Assert(body, "No valid body found");
        if (body)
        {
            AZ::Vector3 beginOfSegments = body->GetPosition();
            return beginOfSegments;
        }
        return AZ::Vector3::CreateZero();
    }
    void ConveyorBeltComponentKinematic::OnEntityActivated(const AZ::EntityId& entityId)
    {
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "No physics system");
        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "No scene interface");
        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene handle");
        m_sceneHandle = defaultSceneHandle;

        m_splineTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(m_splineTransform, m_entity->GetId(), &AZ::TransformBus::Events::GetWorldTM);

        AZ::ConstSplinePtr splinePtr{ nullptr };
        LmbrCentral::SplineComponentRequestBus::EventResult(splinePtr, m_entity->GetId(), &LmbrCentral::SplineComponentRequests::GetSpline);
        AZ_Assert(splinePtr, "Spline pointer is null");

        if (splinePtr)
        {
            m_splineConsPtr = splinePtr;
            m_sceneFinishSimHandler = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
                [this]([[maybe_unused]] AzPhysics::SceneHandle sceneHandle, float fixedDeltaTime)
                {
                    MoveSegmentsPhysically(fixedDeltaTime);
                    SpawnSegments();
                    DespawnSegments();
                },
                aznumeric_cast<int32_t>(AzPhysics::SceneEvents::PhysicsStartFinishSimulationPriority::Components));
            sceneInterface->RegisterSceneSimulationFinishHandler(defaultSceneHandle, m_sceneFinishSimHandler);
            const auto& [startPoint, endPoint] = GetStartAndEndPointOfBelt(splinePtr);
            m_startPoint = startPoint;
            m_endPoint = endPoint;
            m_splineLength = GetSplineLength(splinePtr);

            // initial segment population
            const float normalizedDistanceStep = 0.5f * m_segmentLength / m_splineLength;
            for (float normalizedIndex = 0.f; normalizedIndex < 1.f; normalizedIndex += normalizedDistanceStep)
            {
                m_ConveyorSegments.push_back(CreateSegment(splinePtr, normalizedIndex));
            }
            AZ_Printf("ConveyorBeltComponentKinematic", "Initial Number of segments: %d", m_ConveyorSegments.size());
            AZ::TickBus::Handler::BusConnect();
        }
    }

    AZStd::pair<float, AzPhysics::SimulatedBodyHandle> ConveyorBeltComponentKinematic::CreateSegment(
        AZ::ConstSplinePtr splinePtr, float normalizedLocation)
    {
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();

        auto colliderConfiguration = AZStd::make_shared<Physics::ColliderConfiguration>();
        colliderConfiguration->m_materialSlots.SetMaterialAsset(0, m_materialAsset);
        auto shapeConfiguration =
            AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(m_segmentLength, m_beltWidth, SegmentWidth));
        const auto transform = GetTransformFromSpline(m_splineConsPtr, normalizedLocation);
        AzPhysics::RigidBodyConfiguration conveyorSegmentRigidBodyConfig;
        conveyorSegmentRigidBodyConfig.m_kinematic = true;
        conveyorSegmentRigidBodyConfig.m_position = transform.GetTranslation();
        conveyorSegmentRigidBodyConfig.m_orientation = transform.GetRotation();
        conveyorSegmentRigidBodyConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfiguration, shapeConfiguration);
        conveyorSegmentRigidBodyConfig.m_computeCenterOfMass = true;
        conveyorSegmentRigidBodyConfig.m_computeInertiaTensor = true;
        conveyorSegmentRigidBodyConfig.m_startSimulationEnabled = true;
        conveyorSegmentRigidBodyConfig.m_computeMass = false;
        conveyorSegmentRigidBodyConfig.m_mass = 1.0f;
        conveyorSegmentRigidBodyConfig.m_entityId = GetEntityId();
        conveyorSegmentRigidBodyConfig.m_debugName = "ConveyorBeltSegment";
        AzPhysics::SimulatedBodyHandle handle = physicsSystem->GetScene(m_sceneHandle)->AddSimulatedBody(&conveyorSegmentRigidBodyConfig);
        AZ_Assert(handle == AzPhysics::InvalidSimulatedBodyHandle, "Body created with invalid handle");
        return AZStd::make_pair(normalizedLocation, handle);
    }

    void ConveyorBeltComponentKinematic::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        MoveSegmentsGraphically(deltaTime);
    }

    AZStd::pair<AZ::Vector3, AZ::Vector3> ConveyorBeltComponentKinematic::GetStartAndEndPointOfBelt(AZ::ConstSplinePtr splinePtr)
    {
        AZ::Transform splineTransform = AZ::Transform::Identity();
        AZ::TransformBus::EventResult(splineTransform, m_entity->GetId(), &AZ::TransformBus::Events::GetWorldTM);
        const AZ::SplineAddress addressBegin = splinePtr->GetAddressByFraction(0.f);
        const AZ::SplineAddress addressEnd = splinePtr->GetAddressByFraction(1.f);
        const AZ::Vector3 posBegin = splinePtr->GetPosition(addressBegin);
        const AZ::Vector3 posEnd = splinePtr->GetPosition(addressEnd);
        return { m_splineTransform.TransformPoint(posBegin), m_splineTransform.TransformPoint(posEnd) };
    }

    AZ::Transform ConveyorBeltComponentKinematic::GetTransformFromSpline(AZ::ConstSplinePtr splinePtr, float distanceNormalized)
    {
        AZ_Assert(splinePtr, "Spline pointer is null");
        AZ::SplineAddress address = splinePtr->GetAddressByFraction(distanceNormalized);
        const AZ::Vector3& p = splinePtr->GetPosition(address);

        // construct the rotation matrix from three orthogonal vectors.
        const AZ::Vector3& v1 = splinePtr->GetTangent(address);
        const AZ::Vector3& v2 = splinePtr->GetNormal(address);
        const AZ::Vector3 v3 = v1.Cross(v2);

        AZ::Matrix3x3 rotationMatrix = AZ::Matrix3x3::CreateFromColumns(v1, v2, v3);
        AZ_Assert(rotationMatrix.IsOrthogonal(0.001f), "Rotation matrix is not orthogonal");
        rotationMatrix.Orthogonalize();
        AZ::Transform transform =
            AZ::Transform::CreateFromMatrix3x3AndTranslation(rotationMatrix, p - AZ::Vector3::CreateAxisZ(SegmentWidth / 2.f));
        return m_splineTransform * transform;
    }

    float ConveyorBeltComponentKinematic::GetSplineLength(AZ::ConstSplinePtr splinePtr)
    {
        AZ_Assert(splinePtr, "Spline pointer is null");
        AZ::Transform splineTransform = AZ::Transform::Identity();
        AZ::TransformBus::EventResult(splineTransform, m_entity->GetId(), &AZ::TransformBus::Events::GetWorldTM);
        const AZ::SplineAddress addressEnd = splinePtr->GetAddressByFraction(1.f);
        return splinePtr->GetLength(addressEnd);
    }

    void ConveyorBeltComponentKinematic::MoveSegmentsGraphically(float deltaTime)
    {
        // Animate texture
        AZ::Render::MaterialAssignmentId materialId;
        AZ::Render::MaterialComponentRequestBus::EventResult(
            materialId,
            m_ConveyorEntityId,
            &AZ::Render::MaterialComponentRequestBus::Events::FindMaterialAssignmentId,
            -1,
            m_graphicalMaterialSlot);

        m_textureOffset += deltaTime * m_speed * m_textureScale;

        AZ::Render::MaterialComponentRequestBus::Event(
            m_ConveyorEntityId,
            &AZ::Render::MaterialComponentRequestBus::Events::SetPropertyValueT<float>,
            materialId,
            "uv.offsetU",
            m_textureOffset);
    }

    void ConveyorBeltComponentKinematic::DespawnSegments()
    {
        bool wasSegmentRemoved = false;
        for (auto& [pos, handle] : m_ConveyorSegments)
        {
            if (pos > 1.0f)
            {
                AZ::Interface<AzPhysics::SceneInterface>::Get()->RemoveSimulatedBody(m_sceneHandle, handle);
                handle = AzPhysics::InvalidSimulatedBodyHandle;
                wasSegmentRemoved = true;
            }
        }
        if (wasSegmentRemoved)
        {
            const auto isInvalidHandle = [](const auto& pair)
            {
                return pair.second == AzPhysics::InvalidSimulatedBodyHandle;
            };
            // clear object handle from cache
            m_ConveyorSegments.erase(
                AZStd::remove_if(m_ConveyorSegments.begin(), m_ConveyorSegments.end(), isInvalidHandle), m_ConveyorSegments.end());
        }
    }

    void ConveyorBeltComponentKinematic::MoveSegmentsPhysically(float fixedDeltaTime)
    {
        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        // update positions of the segments
        for (auto& [pos, handle] : m_ConveyorSegments)
        {
            auto* body = azdynamic_cast<AzPhysics::RigidBody*>(sceneInterface->GetSimulatedBodyFromHandle(m_sceneHandle, handle));
            if (body)
            {
                pos += m_speed * fixedDeltaTime / m_splineLength;
                auto transform = GetTransformFromSpline(m_splineConsPtr, pos);
                transform.SetTranslation(transform.GetTranslation());
                body->SetKinematicTarget(transform);
            }
        }
    }

    void ConveyorBeltComponentKinematic::SpawnSegments()
    {
        if (m_ConveyorSegments.empty())
        {
            m_ConveyorSegments.push_back(CreateSegment(m_splineConsPtr, 0.f));
            return;
        }
        AZ::Vector3 position = GetLocationOfSegment(m_ConveyorSegments.front().second);
        AZ::Vector3 distance = m_startPoint - position;
        if (distance.GetLength() > m_segmentLength / 2.f)
        {
            m_ConveyorSegments.push_front(CreateSegment(m_splineConsPtr, 0.f));
        }
    }

} // namespace ROS2
