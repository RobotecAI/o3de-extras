/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Math/Color.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <RobotImporter/URDF/UrdfParser.h>
#include <ignition/math/Pose3.hh>

namespace ROS2
{
    namespace URDF
    {
        //! Common types conversion between urdf and AZ formats
        namespace TypeConversions
        {
            AZ::Vector3 ConvertVector3(const urdf::Vector3& urdfVector);
            AZ::Quaternion ConvertQuaternion(const urdf::Rotation& urdfQuaternion);
            AZ::Color ConvertColor(const urdf::Color& color);
            AZ::Transform ConvertPose(const urdf::Pose& pose);
        } // namespace TypeConversions

    } // namespace URDF

    namespace SDFormat
    {
        //! Common types conversion between urdf and AZ formats
        namespace TypeConversions
        {
            AZ::Vector3 ConvertVector3(const ignition::math::Vector3<double>& sdfVector);
            AZ::Quaternion ConvertQuaternion(const ignition::math::Quaternion<double>& sdfQuaternion);
        } // namespace TypeConversions
    } // namespace SDFormat
} // namespace ROS2
