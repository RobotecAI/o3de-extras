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
    //! Naive "passthrough" projection: publishes the raw cubemap with the six faces laid out side by side
    //! in a horizontal strip (width = 6 x faceSize, height = faceSize), in face index order. No
    //! reprojection - each output pixel is a direct copy of a face pixel - so it is mainly useful for
    //! debugging the rendered faces. All six faces are always rendered.
    class NaiveCubemapConverter : public CubemapConverter
    {
    protected:
        [[nodiscard]] int GetWidth() const override;
        [[nodiscard]] int GetHeight() const override;
        void BuildRemap(const AZStd::array<AZ::Matrix4x4, NumFaces>& faceWorldToClip) override;
    };
} // namespace ROS2Sensors