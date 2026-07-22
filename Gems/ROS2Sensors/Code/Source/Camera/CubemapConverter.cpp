/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "CubemapConverter.h"

#include <AzCore/Math/MathUtils.h>
#include <AzCore/Math/Vector4.h>
#include <cstring>
#include <limits>

namespace ROS2Sensors
{
    void CubemapConverter::Initialize(int faceSize, const AZStd::array<AZ::Matrix4x4, NumFaces>& faceWorldToClip)
    {
        m_faceSize = faceSize;
        m_width = GetWidth();
        m_height = GetHeight();

        const int faceStep = m_faceSize * 4; // RGBA8 bytes per face row
        const size_t pixelCount = static_cast<size_t>(m_width) * m_height;
        m_remapFace.assign(pixelCount, -1);
        m_remapOffset.assign(pixelCount, 0);
        m_faceRequired.fill(false);

        for (int y = 0; y < m_height; ++y)
        {
            for (int x = 0; x < m_width; ++x)
            {
                AZ::Vector3 direction;
                if (!GetSampleDirection(x, y, direction))
                {
                    continue; // Unmapped pixel (left black).
                }
                const AZ::Vector4 direction4(direction.GetX(), direction.GetY(), direction.GetZ(), 0.0f);

                // Pick the cube face that sees this direction most centrally (matches how it was rasterized).
                int bestFace = -1;
                float bestCentering = std::numeric_limits<float>::max();
                float bestNdcX = 0.0f;
                float bestNdcY = 0.0f;
                for (int i = 0; i < NumFaces; ++i)
                {
                    const AZ::Vector4 clip = faceWorldToClip[i] * direction4;
                    const float w = clip.GetW();
                    if (w <= 1e-6f)
                    {
                        continue; // Behind this face's view plane.
                    }
                    const float ndcX = clip.GetX() / w;
                    const float ndcY = clip.GetY() / w;
                    const float centering = AZ::GetMax(std::abs(ndcX), std::abs(ndcY));
                    if (centering <= 1.0f && centering < bestCentering)
                    {
                        bestCentering = centering;
                        bestFace = i;
                        bestNdcX = ndcX;
                        bestNdcY = ndcY;
                    }
                }

                if (bestFace < 0)
                {
                    continue;
                }

                int col = static_cast<int>((bestNdcX * 0.5f + 0.5f) * static_cast<float>(m_faceSize));
                int row = static_cast<int>((0.5f - bestNdcY * 0.5f) * static_cast<float>(m_faceSize));
                col = AZ::GetClamp(col, 0, m_faceSize - 1);
                row = AZ::GetClamp(row, 0, m_faceSize - 1);

                const size_t idx = static_cast<size_t>(y) * m_width + x;
                m_remapFace[idx] = bestFace;
                m_remapOffset[idx] = row * faceStep + col * 4;
                m_faceRequired[bestFace] = true;
            }
        }
    }

    void CubemapConverter::Convert(
        const AZStd::array<AZStd::vector<uint8_t>, NumFaces>& faceData, sensor_msgs::msg::Image& outImage) const
    {
        const uint32_t width = static_cast<uint32_t>(m_width);
        const uint32_t height = static_cast<uint32_t>(m_height);
        const uint32_t step = width * 4; // rgba8

        outImage.width = width;
        outImage.height = height;
        outImage.encoding = "rgba8";
        outImage.step = step;
        outImage.is_bigendian = 0;
        outImage.data.assign(static_cast<size_t>(step) * height, 0);

        const size_t pixelCount = static_cast<size_t>(width) * height;
        for (size_t idx = 0; idx < pixelCount; ++idx)
        {
            const int face = m_remapFace[idx];
            if (face < 0)
            {
                continue;
            }
            const auto& buffer = faceData[face];
            const int offset = m_remapOffset[idx];
            if (offset >= 0 && static_cast<size_t>(offset) + 4 <= buffer.size())
            {
                memcpy(outImage.data.data() + idx * 4, buffer.data() + offset, 4);
            }
        }
    }

    bool CubemapConverter::IsFaceRequired(int faceIndex) const
    {
        return faceIndex >= 0 && faceIndex < NumFaces && m_faceRequired[faceIndex];
    }

    int CubemapConverter::GetOutputWidth() const
    {
        return m_width;
    }

    int CubemapConverter::GetOutputHeight() const
    {
        return m_height;
    }
} // namespace ROS2Sensors
