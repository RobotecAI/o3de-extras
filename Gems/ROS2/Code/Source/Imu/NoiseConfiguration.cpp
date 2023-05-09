/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/EditContext.h>
#include <ROS2/Imu/NoiseConfiguration.h>

namespace ROS2
{
    void NoiseConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            // serializeContext->RegisterGenericType<AZStd::vector<NoiseConfiguration>>();
            serializeContext->Class<NoiseConfiguration>()
                ->Version(1)
                ->Field("ApplyNoise", &NoiseConfiguration::m_applyNoise)
                ->Field("AccelerationVariance", &NoiseConfiguration::m_accelerationVariance)
                ->Field("AngularVelocityVariance", &NoiseConfiguration::m_angularVelocityVariance)
                ->Field("AngleVariance", &NoiseConfiguration::m_angleVariance);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<NoiseConfiguration>("Noise Configuration", "Configuration of IMU noise.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NoiseConfiguration::m_applyNoise, "Apply Noise",
                        "Apply noise to IMU data.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NoiseConfiguration::m_accelerationVariance, "Acceleration Variance",
                        "Variance of acceleration x, y, z.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NoiseConfiguration::m_angularVelocityVariance, "Angular Velocity Variance", 
                        "Variance of angular velocity x, y, z.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NoiseConfiguration::m_angleVariance, "Angle Variance", 
                        "Variance of angle x, y, z.");                    
            }
        }
    }
} // namespace ROS2
