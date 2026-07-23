/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Cubemap/CubemapConverter.h>
#include <Cubemap/EquirectangularConverter.h>
#include <Cubemap/FisheyeConverter.h>

#include <AzCore/Math/MathUtils.h>
#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/Math/Matrix3x4.h>
#include <AzCore/Math/Matrix4x4.h>
#include <AzCore/Math/MatrixUtils.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/algorithm.h>
#include <AzTest/AzTest.h>
#include <cstring>

namespace ROS2Sensors
{
    namespace
    {
        constexpr int FaceSize = 32; // small so face col/row fit in a byte

        // Cube face bases matching ROS2CubemapCameraComponent::MakeFaceBases (right, up, back).
        AZStd::array<AZStd::array<AZ::Vector3, 3>, 6> MakeFaceBases()
        {
            return { { { { AZ::Vector3(0.0f, 1.0f, 0.0f), AZ::Vector3(-1.0f, 0.0f, 0.0f), AZ::Vector3(0.0f, 0.0f, 1.0f) } },
                       { { AZ::Vector3(0.0f, -1.0f, 0.0f), AZ::Vector3(1.0f, 0.0f, 0.0f), AZ::Vector3(0.0f, 0.0f, 1.0f) } },
                       { { AZ::Vector3(-1.0f, 0.0f, 0.0f), AZ::Vector3(0.0f, 0.0f, 1.0f), AZ::Vector3(0.0f, 1.0f, 0.0f) } },
                       { { AZ::Vector3(-1.0f, 0.0f, 0.0f), AZ::Vector3(0.0f, 0.0f, -1.0f), AZ::Vector3(0.0f, -1.0f, 0.0f) } },
                       { { AZ::Vector3(-1.0f, 0.0f, 0.0f), AZ::Vector3(0.0f, -1.0f, 0.0f), AZ::Vector3(0.0f, 0.0f, 1.0f) } },
                       { { AZ::Vector3(1.0f, 0.0f, 0.0f), AZ::Vector3(0.0f, 1.0f, 0.0f), AZ::Vector3(0.0f, 0.0f, 1.0f) } } } };
        }

        // Build the six face world-to-clip matrices in the sensor local frame (identity pose), matching
        // how ROS2CubemapCameraComponent::ComputeFaceWorldToClip derives them. This replicates Atom's
        // View::SetCameraTransform math exactly (see RPI.Public/View.cpp): the camera basis is a Z-up
        // world transform, adjusted to Y-up by a 90-degree X rotation before inverting to world-to-view.
        AZStd::array<AZ::Matrix4x4, 6> BuildFaceMatrices()
        {
            // MakeClipMatrix(faceSize, faceSize, 90, 0.1, 1000): square aspect, reverse depth.
            AZ::Matrix4x4 viewToClip;
            AZ::MakePerspectiveFovMatrixRH(viewToClip, AZ::DegToRad(90.0f), 1.0f, 0.1f, 1000.0f, true);

            const auto bases = MakeFaceBases();
            const AZ::Matrix3x4 zUpToYUp = AZ::Matrix3x4::CreateRotationX(AZ::Constants::HalfPi);

            AZStd::array<AZ::Matrix4x4, 6> worldToClip;
            for (int i = 0; i < 6; ++i)
            {
                // Camera-to-world transform: columns are the face basis [right, up, back].
                AZ::Matrix3x4 cameraTransform;
                cameraTransform.SetBasisAndTranslation(bases[i][0], bases[i][1], bases[i][2], AZ::Vector3::CreateZero());

                const AZ::Matrix3x4 yUpWorld = cameraTransform * zUpToYUp;
                float viewToWorldRaw[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
                yUpWorld.StoreToRowMajorFloat12(viewToWorldRaw);
                const AZ::Matrix4x4 viewToWorld = AZ::Matrix4x4::CreateFromRowMajorFloat16(viewToWorldRaw);
                const AZ::Matrix4x4 worldToView = viewToWorld.GetInverseFast();
                worldToClip[i] = viewToClip * worldToView;
            }
            return worldToClip;
        }

        // Fill each face with a recognisable pattern: pixel (col,row) of face f -> {f, col, row, 255}.
        AZStd::array<AZStd::vector<uint8_t>, 6> MakeSyntheticFaces()
        {
            AZStd::array<AZStd::vector<uint8_t>, 6> faces;
            for (int f = 0; f < 6; ++f)
            {
                faces[f].resize(static_cast<size_t>(FaceSize) * FaceSize * 4);
                for (int row = 0; row < FaceSize; ++row)
                {
                    for (int col = 0; col < FaceSize; ++col)
                    {
                        uint8_t* p = faces[f].data() + (static_cast<size_t>(row) * FaceSize + col) * 4;
                        p[0] = static_cast<uint8_t>(f);
                        p[1] = static_cast<uint8_t>(col);
                        p[2] = static_cast<uint8_t>(row);
                        p[3] = 255;
                    }
                }
            }
            return faces;
        }

        // Emulate the future GPU compute gather: for each output pixel, read (u,v,face,valid) from the
        // deformation buffer and nearest-sample the corresponding face.
        AZStd::vector<uint8_t> GatherViaDeformation(
            const AZStd::vector<float>& deformation,
            const AZStd::array<AZStd::vector<uint8_t>, 6>& faces,
            int width,
            int height)
        {
            AZStd::vector<uint8_t> out(static_cast<size_t>(width) * height * 4, 0);
            const size_t pixelCount = static_cast<size_t>(width) * height;
            for (size_t idx = 0; idx < pixelCount; ++idx)
            {
                const float* d = deformation.data() + idx * 4;
                if (d[3] < 0.5f)
                {
                    continue; // invalid -> black
                }
                int col = static_cast<int>(d[0] * FaceSize);
                int row = static_cast<int>(d[1] * FaceSize);
                col = AZStd::clamp(col, 0, FaceSize - 1);
                row = AZStd::clamp(row, 0, FaceSize - 1);
                const int face = static_cast<int>(d[2] + 0.5f);
                const uint8_t* src = faces[face].data() + (static_cast<size_t>(row) * FaceSize + col) * 4;
                memcpy(out.data() + idx * 4, src, 4);
            }
            return out;
        }

        void ExpectParity(CubemapConverter& converter)
        {
            converter.Initialize(FaceSize, BuildFaceMatrices());
            const auto faces = MakeSyntheticFaces();

            // Reference: the CPU remap the component currently publishes.
            AZStd::vector<uint8_t> reference;
            converter.ConvertToBuffer(faces, reference);

            // Candidate: gather via the baked deformation buffer (what the GPU pass will do).
            const AZStd::vector<float> deformation = converter.BuildDeformationBuffer();
            const int width = converter.GetOutputWidth();
            const int height = converter.GetOutputHeight();

            ASSERT_EQ(deformation.size(), static_cast<size_t>(width) * height * 4);
            const AZStd::vector<uint8_t> candidate = GatherViaDeformation(deformation, faces, width, height);

            ASSERT_EQ(candidate.size(), reference.size());
            EXPECT_EQ(candidate, reference);
        }
    } // namespace

    TEST(CubemapConverterTest, EquirectangularDeformationMatchesCpuConvert)
    {
        EquirectangularConverter converter(64); // 64x32 output
        ExpectParity(converter);
    }

    TEST(CubemapConverterTest, FisheyeDeformationMatchesCpuConvert)
    {
        FisheyeConverter converter(48, 180.0f); // 48x48 output, has invalid (black) corners
        ExpectParity(converter);
    }

    TEST(CubemapConverterTest, EquirectangularUsesAllSixFaces)
    {
        EquirectangularConverter converter(64);
        converter.Initialize(FaceSize, BuildFaceMatrices());
        int required = 0;
        for (int i = 0; i < CubemapConverter::NumFaces; ++i)
        {
            required += converter.IsFaceRequired(i) ? 1 : 0;
        }
        EXPECT_EQ(required, 6);
    }

    TEST(CubemapConverterTest, Fisheye180SkipsRearFace)
    {
        FisheyeConverter converter(48, 180.0f);
        converter.Initialize(FaceSize, BuildFaceMatrices());
        int required = 0;
        for (int i = 0; i < CubemapConverter::NumFaces; ++i)
        {
            required += converter.IsFaceRequired(i) ? 1 : 0;
        }
        // A forward hemisphere never samples the rear face.
        EXPECT_LT(required, 6);
    }

    TEST(CubemapConverterTest, BilinearOnConstantFacesMatchesNearest)
    {
        // Bilerp of equal neighbours is the identity, so on constant-colour faces bilinear must match
        // nearest exactly (the constant where mapped, 0 where unmapped). Exercises the weights summing to
        // one, neighbour indexing, and fixed-point rounding without overflow.
        AZStd::array<AZStd::vector<uint8_t>, 6> faces;
        const uint8_t color[4] = { 17, 89, 200, 255 };
        for (int f = 0; f < 6; ++f)
        {
            faces[f].resize(static_cast<size_t>(FaceSize) * FaceSize * 4);
            for (size_t p = 0; p < faces[f].size(); p += 4)
            {
                memcpy(faces[f].data() + p, color, 4);
            }
        }

        EquirectangularConverter nearest(64);
        nearest.Initialize(FaceSize, BuildFaceMatrices());
        AZStd::vector<uint8_t> nearestOut;
        nearest.ConvertToBuffer(faces, nearestOut);

        EquirectangularConverter bilinear(64);
        bilinear.SetInterpolation(CubemapInterpolation::Bilinear);
        bilinear.Initialize(FaceSize, BuildFaceMatrices());
        AZStd::vector<uint8_t> bilinearOut;
        bilinear.ConvertToBuffer(faces, bilinearOut);

        ASSERT_EQ(bilinearOut.size(), nearestOut.size());
        EXPECT_EQ(bilinearOut, nearestOut);
    }
} // namespace ROS2Sensors
