/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    //! Configuration for handling of IMU noise.
    struct NoiseConfiguration
    {
    public:
        AZ_TYPE_INFO(NoiseConfiguration, "{4B119856-6DD6-449D-9D42-4319FFDC8808}");
        static void Reflect(AZ::ReflectContext* context);

        bool m_applyNoise = false;
        AZ::Vector3 m_accelerationVariance;
        AZ::Vector3 m_angularVelocityVariance;
        AZ::Vector3 m_angleVariance;
    };
} // namespace ROS2
