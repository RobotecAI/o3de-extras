/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "NaiveCubemapConverter.h"

namespace ROS2Sensors
{
    int NaiveCubemapConverter::GetWidth() const
    {
        return GetFaceSize() * NumFaces;
    }

    int NaiveCubemapConverter::GetHeight() const
    {
        return GetFaceSize();
    }

    void NaiveCubemapConverter::BuildRemap([[maybe_unused]] const AZStd::array<AZ::Matrix4x4, NumFaces>& faceWorldToClip)
    {
        const int faceSize = GetFaceSize();
        for (int face = 0; face < NumFaces; ++face)
        {
            const int xOffset = face * faceSize;
            for (int row = 0; row < faceSize; ++row)
            {
                for (int col = 0; col < faceSize; ++col)
                {
                    SetPixelSource(xOffset + col, row, face, col, row);
                }
            }
        }
    }
} // namespace ROS2Sensors