/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Matrix4x4.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/array.h>
#include <AzCore/std/containers/vector.h>
#include <sensor_msgs/msg/image.hpp>

namespace ROS2Sensors
{
    //! Base class for a projection model that converts a rendered cubemap (six square RGBA8 faces) into
    //! a single "ready" output image (e.g. equirectangular, fisheye).
    //!
    //! The base class owns everything shared between projections: it precomputes a lookup table mapping
    //! every output pixel to a source cube face + face pixel, tracks which faces are actually sampled,
    //! and assembles the output image. A concrete model only implements the projection virtuals below,
    //! which decide the output size and, per output pixel, the direction (and therefore what is rendered).
    class CubemapConverter
    {
    public:
        static constexpr int NumFaces = 6;

        virtual ~CubemapConverter() = default;

        //! Precompute the remap table. Must be called once before Convert().
        //! @param faceSize per-face resolution in pixels (square).
        //! @param faceWorldToClip world-to-clip matrix of each cube face, in the sensor's local frame.
        void Initialize(int faceSize, const AZStd::array<AZ::Matrix4x4, NumFaces>& faceWorldToClip);

        //! Fill @p outImage (RGBA8) from the six face readback buffers using the precomputed table.
        //! Sets width/height/encoding/step/data; the caller fills the header and publishes.
        void Convert(const AZStd::array<AZStd::vector<uint8_t>, NumFaces>& faceData, sensor_msgs::msg::Image& outImage) const;

        //! Whether a given cube face is sampled by this projection (valid after Initialize). Lets the
        //! caller skip rendering faces that never contribute to the output.
        [[nodiscard]] bool IsFaceRequired(int faceIndex) const;

        [[nodiscard]] int GetOutputWidth() const;
        [[nodiscard]] int GetOutputHeight() const;

    protected:
        //! --- Projection model: implemented by derived classes ---

        //! Output image dimensions in pixels.
        [[nodiscard]] virtual int GetWidth() const = 0;
        [[nodiscard]] virtual int GetHeight() const = 0;

        //! Direction sampled by output pixel (x, y), in the sensor's local frame (X right, Y forward,
        //! Z up). Return false to leave the pixel unmapped/black (e.g. outside a fisheye circle).
        virtual bool GetSampleDirection(int x, int y, AZ::Vector3& outDirection) const = 0;

    private:
        int m_faceSize = 0;
        int m_width = 0;
        int m_height = 0;

        // For each output pixel: source face index (-1 if unmapped) and byte offset into that face buffer.
        AZStd::vector<int32_t> m_remapFace;
        AZStd::vector<int32_t> m_remapOffset;
        AZStd::array<bool, NumFaces> m_faceRequired{};
    };
} // namespace ROS2Sensors
