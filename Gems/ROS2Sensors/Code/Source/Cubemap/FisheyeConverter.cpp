/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FisheyeConverter.h"

#include <AzCore/Math/MathUtils.h>
#include <cmath>

namespace ROS2Sensors
{
    int FisheyeConverter::GetWidth() const
    {
        return m_size;
    }

    int FisheyeConverter::GetHeight() const
    {
        return m_size;
    }

    bool FisheyeConverter::GetSampleDirection(int x, int y, AZ::Vector3& outDirection) const
    {
        const float size = static_cast<float>(m_size);
        // Normalised image coordinates in [-1, 1]; the centre is the optical axis.
        const float nx = (static_cast<float>(x) + 0.5f) / size * 2.0f - 1.0f;
        const float ny = (static_cast<float>(y) + 0.5f) / size * 2.0f - 1.0f;
        const float radius = std::sqrt(nx * nx + ny * ny);
        if (radius > 1.0f)
        {
            return false; // Outside the image circle -> left black.
        }

        // Equidistant fisheye: angle from the forward axis is proportional to the image radius.
        const float theta = radius * 0.5f * AZ::DegToRad(m_fovDeg);
        if (radius < 1e-6f)
        {
            outDirection.Set(0.0f, 1.0f, 0.0f); // Optical axis = sensor forward.
            return true;
        }

        // Sensor local frame: X right, Y forward, Z up. Image +x -> right, image top (-ny) -> up.
        const float sinTheta = std::sin(theta);
        outDirection.Set(nx / radius * sinTheta, std::cos(theta), -ny / radius * sinTheta);
        return true;
    }
} // namespace ROS2Sensors
