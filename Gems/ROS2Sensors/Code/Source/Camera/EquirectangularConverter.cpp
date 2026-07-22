/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "EquirectangularConverter.h"

#include <AzCore/Math/MathUtils.h>
#include <cmath>

namespace ROS2Sensors
{
    int EquirectangularConverter::GetWidth() const
    {
        return m_width;
    }

    int EquirectangularConverter::GetHeight() const
    {
        return m_width / 2;
    }

    bool EquirectangularConverter::GetSampleDirection(int x, int y, AZ::Vector3& outDirection) const
    {
        const float width = static_cast<float>(GetWidth());
        const float height = static_cast<float>(GetHeight());

        // Latitude: +pi/2 at the top row (zenith) to -pi/2 at the bottom row.
        const float lat = AZ::Constants::HalfPi - (static_cast<float>(y) + 0.5f) / height * AZ::Constants::Pi;
        // Longitude: -pi..pi across the width; 0 (image centre) = sensor forward.
        const float lon = (static_cast<float>(x) + 0.5f) / width * AZ::Constants::TwoPi - AZ::Constants::Pi;

        // Sensor local frame: X right, Y forward, Z up.
        outDirection.Set(std::sin(lon) * std::cos(lat), std::cos(lon) * std::cos(lat), std::sin(lat));
        return true;
    }
} // namespace ROS2Sensors
