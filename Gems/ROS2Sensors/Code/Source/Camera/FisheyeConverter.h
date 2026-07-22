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
    //! Equidistant fisheye projection into a circular image on a square canvas. The optical axis is the
    //! sensor's forward (+Y); the angle from that axis is proportional to the image radius. Pixels outside
    //! the image circle are left black. A FOV <= 180 only samples the forward hemisphere, so the rear cube
    //! face is never rendered.
    class FisheyeConverter : public CubemapConverter
    {
    public:
        //! @param size output width and height in pixels (square).
        //! @param fovDeg full field of view of the fisheye lens in degrees (e.g. 180).
        FisheyeConverter(int size, float fovDeg)
            : m_size(size)
            , m_fovDeg(fovDeg)
        {
        }

    protected:
        [[nodiscard]] int GetWidth() const override;
        [[nodiscard]] int GetHeight() const override;
        bool GetSampleDirection(int x, int y, AZ::Vector3& outDirection) const override;

    private:
        int m_size = 0;
        float m_fovDeg = 180.0f;
    };
} // namespace ROS2Sensors
