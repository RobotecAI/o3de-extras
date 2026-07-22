/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "CubemapConverter.h"

namespace ROS2Sensors
{
    //! Equirectangular (2:1 lat-long) projection. Forward is at the image centre, zenith at the top row.
    class EquirectangularConverter : public CubemapConverter
    {
    public:
        //! @param width output width in pixels; height is half of this.
        explicit EquirectangularConverter(int width)
            : m_width(width)
        {
        }

    protected:
        [[nodiscard]] int GetWidth() const override;
        [[nodiscard]] int GetHeight() const override;
        bool GetSampleDirection(int x, int y, AZ::Vector3& outDirection) const override;

    private:
        int m_width = 0;
    };
} // namespace ROS2Sensors
