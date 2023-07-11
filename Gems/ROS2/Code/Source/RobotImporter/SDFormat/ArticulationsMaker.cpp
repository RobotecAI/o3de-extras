/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ArticulationsMaker.h"
#include "RobotImporter/Utils/DefaultSolverConfiguration.h"
#include <AzCore/std/containers/unordered_map.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <PhysX/ArticulationTypes.h>
#include <RobotImporter/Utils/TypeConversions.h>
#include <Source/EditorArticulationLinkComponent.h>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>

namespace ROS2::SDFormat
{

    namespace
    {
        using ArticulationCfg = PhysX::EditorArticulationLinkConfiguration;
        static const AZStd::unordered_map<sdf::JointType, PhysX::ArticulationJointType> SupportedJointTypes{ {
            { sdf::JointType::REVOLUTE, PhysX::ArticulationJointType::Hinge },
            { sdf::JointType::CONTINUOUS, PhysX::ArticulationJointType::Hinge },
            { sdf::JointType::PRISMATIC, PhysX::ArticulationJointType::Prismatic },
            { sdf::JointType::FIXED, PhysX::ArticulationJointType::Fix },
        } };
    } // namespace

    void AddToArticulationConfig(ArticulationCfg& articulationLinkConfiguration, const sdf::Joint* joint)
    {
        if (!joint)
        {
            return;
        }
        auto supportedArticulationType = SupportedJointTypes.find(joint->Type());
        AZ_Warning(
            "ArticulationsMaker",
            supportedArticulationType != SupportedJointTypes.end(),
            "Articulations do not support type %d for SDFormat joint %s.",
            joint->Type(),
            joint->Name().c_str());
        if (supportedArticulationType != SupportedJointTypes.end())
        {
            const auto type = supportedArticulationType->second;
            articulationLinkConfiguration.m_articulationJointType = type;
            const AZ::Vector3 o3deJointDir{ 1.0, 0.0, 0.0 };
            const sdf::JointAxis* axis = joint->Axis(0);
            ignition::math::Vector3d axisXyz;
            sdf::Errors errors = axis->ResolveXyz(axisXyz, joint->Name());
            AZ_Error(
                "ArticulationsMaker",
                !errors.empty(),
                "Cannot resolve xyz axis for SDFormat joint %s.",
                joint->Type(),
                joint->Name().c_str());

            const AZ::Vector3 jointAxis = TypeConversions::ConvertVector3(axisXyz);
            const auto quaternion =
                jointAxis.IsZero() ? AZ::Quaternion::CreateIdentity() : AZ::Quaternion::CreateShortestArc(o3deJointDir, jointAxis);
            const AZ::Vector3 rotation = quaternion.GetEulerDegrees();
            articulationLinkConfiguration.m_localRotation = rotation;

            if (type == PhysX::ArticulationJointType::Hinge)
            {
                const double limitUpper = AZ::RadToDeg(axis->Upper());
                const double limitLower = AZ::RadToDeg(axis->Lower());
                articulationLinkConfiguration.m_angularLimitNegative = limitLower;
                articulationLinkConfiguration.m_angularLimitPositive = limitUpper;
            }
            else if (type == PhysX::ArticulationJointType::Prismatic)
            {
                articulationLinkConfiguration.m_linearLimitLower = axis->Upper();
                articulationLinkConfiguration.m_linearLimitUpper = axis->Lower();
            }
            else
            {
                articulationLinkConfiguration.m_isLimited = false;
            }
        }
    }

    void AddToArticulationConfig(
        ArticulationCfg& articulationLinkConfiguration, const sdf::Joint* joint, const ignition::math::Inertiald& inertial)
    {
        articulationLinkConfiguration.m_solverPositionIterations =
            AZStd::max(articulationLinkConfiguration.m_solverPositionIterations, DefaultNumberPosSolver);
        articulationLinkConfiguration.m_solverVelocityIterations =
            AZStd::max(articulationLinkConfiguration.m_solverVelocityIterations, DefaultNumberVelSolver);

        articulationLinkConfiguration.m_mass = inertial.MassMatrix().Mass();
        articulationLinkConfiguration.m_centerOfMassOffset = TypeConversions::ConvertVector3(inertial.Pose().Pos());

        // There is a rotation component in SDFormat that we are not able to apply
        AZ_Warning(
            "AddArticulationLink",
            TypeConversions::ConvertQuaternion(inertial.Pose().Rot()).IsIdentity(),
            "Ignoring SDFormat inertial origin rotation (no such field in rigid body configuration)");
    }

    void ArticulationsMaker::AddArticulationLink(const sdf::Link* link, const sdf::Joint* parentJoint, AZ::EntityId entityId) const
    {
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        AZ_Assert(entity, "No entity for id %s", entityId.ToString().c_str());

        AZ_TracePrintf("ArticulationsMaker", "Processing inertial for entity id: %s\n", entityId.ToString().c_str());
        PhysX::EditorArticulationLinkConfiguration articulationLinkConfiguration;

        AddToArticulationConfig(articulationLinkConfiguration, parentJoint, link->Inertial());
        AddToArticulationConfig(articulationLinkConfiguration, parentJoint);

        entity->CreateComponent<PhysX::EditorArticulationLinkComponent>(articulationLinkConfiguration);
    }
} // namespace ROS2::SDFormat
