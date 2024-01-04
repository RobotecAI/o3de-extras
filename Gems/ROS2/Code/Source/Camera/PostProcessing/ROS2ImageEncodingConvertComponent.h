/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/RTTI/TypeInfoSimple.h>
#include <ROS2/Camera/CameraPostProcessingRequestBus.h>

namespace ROS2
{
    enum class ImageEncoding : AZ::u8
    {
        rgba8,
        rgb8,
        mono8,
        mono16,
    };

    struct EncodingConverion
    {
        AZ_TYPE_INFO(EncodingConverion, "{db361adc-b339-4a4e-a10b-c6bf6791eda6}");
        static void Reflect(AZ::ReflectContext* context);

        bool operator==(const ROS2::EncodingConverion& rhs) const
        {
            return encodingIn == rhs.encodingIn && encodingOut == rhs.encodingOut;
        }

        ImageEncoding encodingIn = ImageEncoding::rgba8;
        ImageEncoding encodingOut = ImageEncoding::rgb8;
    };

    //! Change image format
    class ROS2ImageEncodingConvertComponent
        : public AZ::Component
        , public CameraPostProcessingRequestBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2ImageEncodingConvertComponent, "12449810-d179-44f1-8f72-22d8d3fa4460");
        static void Reflect(AZ::ReflectContext* context);

        ROS2ImageEncodingConvertComponent() = default;
        ~ROS2ImageEncodingConvertComponent() override = default;

        void Activate() override;
        void Deactivate() override;

        //! CameraPostProcessingRequestBus::Handler overrides
        void ApplyPostProcessing(sensor_msgs::msg::Image& image) override;
        AZ::u8 GetPriority() const override;

        AZ::Outcome<void, AZStd::string> ValidateEncodingConversion(void* newValue, const AZ::Uuid& valueType);

    private:
        AZ::u8 m_priority = CameraPostProcessingRequests::DEFAULT_PRIORITY;
        EncodingConverion m_encodingConvertData;
    };
} // namespace ROS2
