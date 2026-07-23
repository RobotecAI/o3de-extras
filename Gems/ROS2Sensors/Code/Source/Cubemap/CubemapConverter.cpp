/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "CubemapConverter.h"

#include <AzCore/Debug/Profiler.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Math/Vector4.h>
#include <cmath>
#include <cstring>
#include <limits>

AZ_DEFINE_BUDGET(ROS2Sensors);

namespace ROS2Sensors
{
    void CubemapConverter::SetInterpolation(CubemapInterpolation mode)
    {
        m_interpolation = mode;
    }

    void CubemapConverter::Initialize(int faceSize, const AZStd::array<AZ::Matrix4x4, NumFaces>& faceWorldToClip)
    {
        m_faceSize = faceSize;
        m_width = GetWidth();
        m_height = GetHeight();

        const size_t pixelCount = static_cast<size_t>(m_width) * m_height;
        m_remapFace.assign(pixelCount, -1);
        m_remapOffset.assign(pixelCount, 0);
        m_faceRequired.fill(false);
        if (m_interpolation == CubemapInterpolation::Bilinear)
        {
            m_bilinear.assign(pixelCount, BilinearTap{});
        }
        else
        {
            m_bilinear.clear();
        }

        BuildRemap(faceWorldToClip);
    }

    void CubemapConverter::BuildRemap(const AZStd::array<AZ::Matrix4x4, NumFaces>& faceWorldToClip)
    {
        AZ_PROFILE_SCOPE(ROS2Sensors, "CubemapConverter::BuildRemap");
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
                // Select purely by smallest centering - do NOT gate on centering <= 1: a direction exactly
                // on a face seam projects to centering ~= 1 on both adjacent faces, and float error can push
                // both just over 1, which would leave the pixel unmapped and paint a black line along every
                // cube seam. The most-central face is always the correct one; the col/row clamp below absorbs
                // the sub-texel overshoot at the edge.
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
                    if (centering < bestCentering)
                    {
                        bestCentering = centering;
                        bestFace = i;
                        bestNdcX = ndcX;
                        bestNdcY = ndcY;
                    }
                }

                if (bestFace < 0)
                {
                    continue; // No face in front of this direction (should not happen for a valid sample).
                }

                int col = static_cast<int>((bestNdcX * 0.5f + 0.5f) * static_cast<float>(m_faceSize));
                int row = static_cast<int>((0.5f - bestNdcY * 0.5f) * static_cast<float>(m_faceSize));
                col = AZ::GetClamp(col, 0, m_faceSize - 1);
                row = AZ::GetClamp(row, 0, m_faceSize - 1);

                SetPixelSource(x, y, bestFace, col, row);

                if (m_interpolation == CubemapInterpolation::Bilinear)
                {
                    const size_t idx = static_cast<size_t>(y) * m_width + x;
                    m_bilinear[idx] = MakeBilinearTap(bestNdcX, bestNdcY);
                }
            }
        }
    }

    CubemapConverter::BilinearTap CubemapConverter::MakeBilinearTap(float ndcX, float ndcY) const
    {
        const int faceStep = m_faceSize * 4; // RGBA8 bytes per face row

        // Continuous texel-centre position of the sample within the face.
        const float px = (ndcX * 0.5f + 0.5f) * static_cast<float>(m_faceSize) - 0.5f;
        const float py = (0.5f - ndcY * 0.5f) * static_cast<float>(m_faceSize) - 0.5f;

        int col = static_cast<int>(std::floor(px));
        int rowY = static_cast<int>(std::floor(py));
        float fu = px - static_cast<float>(col);
        float fv = py - static_cast<float>(rowY);

        // Edge clamp: at a face boundary the neighbour step is zeroed so the tap repeats the edge texel
        // instead of reaching into an adjacent (unmapped) face.
        int du = 4;
        int dv = faceStep;
        if (col < 0)
        {
            col = 0;
            fu = 0.0f;
            du = 0;
        }
        else if (col >= m_faceSize - 1)
        {
            col = m_faceSize - 1;
            fu = 0.0f;
            du = 0;
        }
        if (rowY < 0)
        {
            rowY = 0;
            fv = 0.0f;
            dv = 0;
        }
        else if (rowY >= m_faceSize - 1)
        {
            rowY = m_faceSize - 1;
            fv = 0.0f;
            dv = 0;
        }

        BilinearTap tap;
        tap.m_base = rowY * faceStep + col * 4;
        tap.m_du = du;
        tap.m_dv = dv;
        tap.m_wu = static_cast<uint16_t>(fu * 256.0f + 0.5f);
        tap.m_wv = static_cast<uint16_t>(fv * 256.0f + 0.5f);
        return tap;
    }

    void CubemapConverter::SetPixelSource(int x, int y, int face, int faceCol, int faceRow)
    {
        if (face < 0 || face >= NumFaces || x < 0 || x >= m_width || y < 0 || y >= m_height)
        {
            return;
        }
        const int faceStep = m_faceSize * 4; // RGBA8 bytes per face row
        const size_t idx = static_cast<size_t>(y) * m_width + x;
        m_remapFace[idx] = face;
        m_remapOffset[idx] = faceRow * faceStep + faceCol * 4;
        m_faceRequired[face] = true;

        // Default the bilinear tap to the same single texel (zero weights) so direct layouts (e.g. the
        // naive grid) reduce to nearest under bilinear mode. BuildRemap overwrites this for reprojected
        // pixels that have a fractional sample position.
        if (!m_bilinear.empty())
        {
            BilinearTap tap;
            tap.m_base = m_remapOffset[idx];
            m_bilinear[idx] = tap;
        }
    }

    int CubemapConverter::GetFaceSize() const
    {
        return m_faceSize;
    }

    void CubemapConverter::ConvertToBuffer(
        const AZStd::array<AZStd::vector<uint8_t>, NumFaces>& faceData, AZStd::vector<uint8_t>& outData) const
    {
        AZ_PROFILE_SCOPE(ROS2Sensors, "CubemapConverter::ConvertToBuffer");

        const size_t step = static_cast<size_t>(m_width) * 4; // rgba8
        outData.assign(step * m_height, 0);

        const size_t pixelCount = static_cast<size_t>(m_width) * m_height;
        const bool bilinear = (m_interpolation == CubemapInterpolation::Bilinear) && (m_bilinear.size() == pixelCount);

        for (size_t idx = 0; idx < pixelCount; ++idx)
        {
            const int face = m_remapFace[idx];
            if (face < 0)
            {
                continue;
            }
            const auto& buffer = faceData[face];

            if (!bilinear)
            {
                const int offset = m_remapOffset[idx];
                if (offset >= 0 && static_cast<size_t>(offset) + 4 <= buffer.size())
                {
                    memcpy(outData.data() + idx * 4, buffer.data() + offset, 4);
                }
                continue;
            }

            const BilinearTap& tap = m_bilinear[idx];
            const size_t maxOffset = static_cast<size_t>(tap.m_base) + tap.m_du + tap.m_dv;
            if (tap.m_base < 0 || maxOffset + 4 > buffer.size())
            {
                continue;
            }
            const uint8_t* p00 = buffer.data() + tap.m_base;
            const uint8_t* p10 = p00 + tap.m_du;
            const uint8_t* p01 = p00 + tap.m_dv;
            const uint8_t* p11 = p01 + tap.m_du;
            uint8_t* out = outData.data() + idx * 4;
            const uint32_t wu = tap.m_wu;
            const uint32_t wv = tap.m_wv;
            for (int c = 0; c < 4; ++c)
            {
                const uint32_t top = static_cast<uint32_t>(p00[c]) * (256 - wu) + static_cast<uint32_t>(p10[c]) * wu;
                const uint32_t bot = static_cast<uint32_t>(p01[c]) * (256 - wu) + static_cast<uint32_t>(p11[c]) * wu;
                out[c] = static_cast<uint8_t>((top * (256 - wv) + bot * wv) >> 16);
            }
        }
    }

    void CubemapConverter::Convert(
        const AZStd::array<AZStd::vector<uint8_t>, NumFaces>& faceData, sensor_msgs::msg::Image& outImage) const
    {
        outImage.width = static_cast<uint32_t>(m_width);
        outImage.height = static_cast<uint32_t>(m_height);
        outImage.encoding = "rgba8";
        outImage.step = static_cast<uint32_t>(m_width) * 4;
        outImage.is_bigendian = 0;

        AZStd::vector<uint8_t> buffer;
        ConvertToBuffer(faceData, buffer);
        outImage.data.assign(buffer.begin(), buffer.end());
    }

    AZStd::vector<float> CubemapConverter::BuildDeformationBuffer() const
    {
        const int faceStep = m_faceSize * 4; // RGBA8 bytes per face row
        const size_t pixelCount = static_cast<size_t>(m_width) * m_height;
        AZStd::vector<float> buffer(pixelCount * 4, 0.0f);

        for (size_t idx = 0; idx < pixelCount; ++idx)
        {
            float* texel = buffer.data() + idx * 4;
            const int face = m_remapFace[idx];
            if (face < 0)
            {
                texel[3] = 0.0f; // invalid -> gathered as black on the GPU
                continue;
            }
            const int offset = m_remapOffset[idx];
            const int col = (offset % faceStep) / 4;
            const int row = offset / faceStep;
            texel[0] = (static_cast<float>(col) + 0.5f) / static_cast<float>(m_faceSize); // u
            texel[1] = (static_cast<float>(row) + 0.5f) / static_cast<float>(m_faceSize); // v
            texel[2] = static_cast<float>(face);
            texel[3] = 1.0f;
        }
        return buffer;
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
