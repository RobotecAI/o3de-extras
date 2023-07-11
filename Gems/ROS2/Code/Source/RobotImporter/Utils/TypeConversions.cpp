/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "TypeConversions.h"

namespace ROS2
{
    namespace URDF
    {
        AZ::Vector3 TypeConversions::ConvertVector3(const urdf::Vector3& urdfVector)
        {
            return AZ::Vector3(urdfVector.x, urdfVector.y, urdfVector.z);
        }

        AZ::Quaternion TypeConversions::ConvertQuaternion(const urdf::Rotation& urdfQuaternion)
        {
            return AZ::Quaternion(urdfQuaternion.x, urdfQuaternion.y, urdfQuaternion.z, urdfQuaternion.w);
        }

        AZ::Color TypeConversions::ConvertColor(const urdf::Color& color)
        {
            return AZ::Color(color.r, color.g, color.b, color.a);
        }

        AZ::Transform TypeConversions::ConvertPose(const urdf::Pose& pose)
        {
            AZ::Quaternion azRotation = URDF::TypeConversions::ConvertQuaternion(pose.rotation);
            AZ::Vector3 azPosition = URDF::TypeConversions::ConvertVector3(pose.position);
            return AZ::Transform(azPosition, azRotation, 1.0f);
        }
    } // namespace URDF

    namespace SDFormat
    {
        AZ::Vector3 TypeConversions::ConvertVector3(const ignition::math::Vector3<double>& sdfVector)
        {
            return AZ::Vector3(sdfVector.X(), sdfVector.Y(), sdfVector.Z());
        }

        AZ::Quaternion TypeConversions::ConvertQuaternion(const ignition::math::Quaternion<double>& sdfQuaternion)
        {
            return AZ::Quaternion(sdfQuaternion.X(), sdfQuaternion.Y(), sdfQuaternion.Z(), sdfQuaternion.W());
        }
    } // namespace SDFormat
} // namespace ROS2
