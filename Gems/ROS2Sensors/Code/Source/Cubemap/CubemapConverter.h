/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Debug/Budget.h>
#include <AzCore/Math/Matrix4x4.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/base.h>
#include <AzCore/std/containers/array.h>
#include <AzCore/std/containers/vector.h>
#include <cstdint>
#include <sensor_msgs/msg/image.hpp>

//! Profiling budget for the cubemap camera (assembly + publish). Defined once in CubemapConverter.cpp.
AZ_DECLARE_BUDGET(ROS2Sensors);

namespace ROS2Sensors
{
    //! How the output image samples the rendered cube faces.
    enum class CubemapInterpolation : AZ::u8
    {
        Nearest, //!< One source texel per output pixel (fast, aliased/blocky).
        Bilinear //!< 4-tap within-face blend (smoother; edge-clamped at face seams).
    };

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

        //! Select nearest (default) or bilinear sampling. Call before Initialize() - it changes what the
        //! remap table stores.
        void SetInterpolation(CubemapInterpolation mode);

        //! Precompute the remap table. Must be called once before Convert().
        //! @param faceSize per-face resolution in pixels (square).
        //! @param faceWorldToClip world-to-clip matrix of each cube face, in the sensor's local frame.
        void Initialize(int faceSize, const AZStd::array<AZ::Matrix4x4, NumFaces>& faceWorldToClip);

        //! Fill @p outImage (RGBA8) from the six face readback buffers using the precomputed table.
        //! Sets width/height/encoding/step/data; the caller fills the header and publishes.
        void Convert(const AZStd::array<AZStd::vector<uint8_t>, NumFaces>& faceData, sensor_msgs::msg::Image& outImage) const;

        //! Core RGBA8 remap (no ROS types): gather every output pixel from the face buffers via the
        //! precomputed table. @p outData is resized to width*height*4. This is what Convert() wraps.
        void ConvertToBuffer(const AZStd::array<AZStd::vector<uint8_t>, NumFaces>& faceData, AZStd::vector<uint8_t>& outData) const;

        //! Bake the remap table into a "deformation" buffer for GPU assembly: width*height RGBA32F texels,
        //! each holding (u, v, faceIndex, valid) - source UV within a face, face index, and a 0/1 flag.
        //! A compute/fullscreen pass can then gather the output image from the face textures on the GPU.
        //! Valid after Initialize.
        [[nodiscard]] AZStd::vector<float> BuildDeformationBuffer() const;

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
        //! Z up). Return false to leave the pixel unmapped/black (e.g. outside a fisheye circle). Used
        //! by the default BuildRemap; direct layouts that override BuildRemap can ignore this.
        virtual bool GetSampleDirection([[maybe_unused]] int x, [[maybe_unused]] int y, [[maybe_unused]] AZ::Vector3& outDirection) const
        {
            return false;
        }

        //! Populate the remap table (called by Initialize once the output size is known). The default
        //! reprojects: for each output pixel it takes GetSampleDirection and finds the cube face that
        //! rasterized it. Override for a direct layout (e.g. a raw cubemap grid) that copies face pixels
        //! without reprojection.
        virtual void BuildRemap(const AZStd::array<AZ::Matrix4x4, NumFaces>& faceWorldToClip);

        //! For BuildRemap overrides: point output pixel (x, y) at pixel (faceCol, faceRow) of @p face.
        void SetPixelSource(int x, int y, int face, int faceCol, int faceRow);

        //! Per-face resolution in pixels (square). Valid from the start of BuildRemap / GetWidth / GetHeight.
        [[nodiscard]] int GetFaceSize() const;

    private:
        //! Precomputed 4-tap bilinear source for one output pixel, within a single face. The three steps
        //! (du/dv) are zero at a face edge so the taps clamp instead of crossing into another face.
        struct BilinearTap
        {
            int32_t m_base = 0; //!< byte offset of the top-left texel in the face buffer
            int32_t m_du = 0; //!< byte step to the right neighbour (0 at the right edge)
            int32_t m_dv = 0; //!< byte step to the bottom neighbour (0 at the bottom edge)
            uint16_t m_wu = 0; //!< horizontal weight, fixed-point in [0,256]
            uint16_t m_wv = 0; //!< vertical weight, fixed-point in [0,256]
        };

        //! Build the 4-tap bilinear source for the sample at face NDC (ndcX, ndcY). Valid during BuildRemap.
        [[nodiscard]] BilinearTap MakeBilinearTap(float ndcX, float ndcY) const;

        int m_faceSize = 0;
        int m_width = 0;
        int m_height = 0;
        CubemapInterpolation m_interpolation = CubemapInterpolation::Nearest;

        // For each output pixel: source face index (-1 if unmapped) and byte offset into that face buffer.
        AZStd::vector<int32_t> m_remapFace;
        AZStd::vector<int32_t> m_remapOffset;
        // Per output pixel bilinear taps (only populated when m_interpolation == Bilinear).
        AZStd::vector<BilinearTap> m_bilinear;
        AZStd::array<bool, NumFaces> m_faceRequired{};
    };
} // namespace ROS2Sensors
