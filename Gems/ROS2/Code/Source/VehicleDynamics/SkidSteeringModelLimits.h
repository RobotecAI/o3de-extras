/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/RTTI/TypeInfo.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2::VehicleDynamics
{
    //! A structure holding limits of skid-steering robot.
    struct SkidSteeringModelLimits
    {
    public:
        AZ_TYPE_INFO(SkidSteeringModelLimits, "{23420EFB-BB62-48C7-AD37-E50580A53C39}");
        SkidSteeringModelLimits() = default;
        static void Reflect(AZ::ReflectContext* context);

        //! Limit value with a symmetrical range.
        //! @param value Input value.
        //! @param absoluteLimit Limits for value (between -absoluteLimit and absoluteLimit).
        //! @returns A limited value. Always returns either value, -absoluteLimit or absoluteLimit.
        static float LimitValue(float value, float absoluteLimit);

        float m_linearLimit = 1.0f;
        float m_angularLimit = 1.0f;
    };
} // namespace ROS2::VehicleDynamics
