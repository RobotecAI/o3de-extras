/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    //! Configuration for handling of IMU noise.
    struct NoiseConfiguration
    {
    public:
        AZ_TYPE_INFO(NoiseConfiguration, "{E0B0F6A0-0F28-46D5-95F1-956550BA97B9}");
        static void Reflect(AZ::ReflectContext* context);

        bool m_applyNoise;
        AZ::Vector3 m_accelerationVariance; // Variance of acceleration x, y, z
        AZ::Vector3 m_angularVelocityVariance; // Variance of angular velocity x, y, z
        AZ::Vector3 m_angleVariance; // Variance of angle x, y, z
    };
} // namespace ROS2
