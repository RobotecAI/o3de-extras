/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2ImageEncodingConvertComponent.h"
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <sensor_msgs/msg/detail/image__struct.hpp>

namespace AZStd
{
    template<>
    struct hash<ROS2::EncodingConverion>
    {
        size_t operator()(const ROS2::EncodingConverion& data) const
        {
            return (static_cast<AZ::u16>(data.encodingIn) << 8) | static_cast<AZ::u16>(data.encodingOut);
        }
    };
} // namespace AZStd

namespace ROS2
{
    namespace
    {
        void Rgba8ToRgb8(sensor_msgs::msg::Image& image)
        {
            AZ_Assert(image.encoding == "rgba8", "Image encoding is not rgba8");
            AZ_Assert(image.step == image.width * 4, "Image step is not width * 4");
            AZ_Assert(image.data.size() == image.step * image.height, "Image data size is not step * height");

            // Perform conversion in place
            for (size_t pixelId = 0; pixelId < image.width * image.height; ++pixelId)
            {
                size_t pixelOffsetIn = pixelId * 4;
                size_t pixelOffsetOut = pixelId * 3;
                image.data[pixelOffsetOut] = image.data[pixelOffsetIn];
                image.data[pixelOffsetOut + 1] = image.data[pixelOffsetIn + 1];
                image.data[pixelOffsetOut + 2] = image.data[pixelOffsetIn + 2];
            }
            image.encoding = "rgb8";
            image.step = image.width * 3;
            image.data.resize(image.step * image.height);
        }

        const AZStd::unordered_map<ImageEncoding, const char*> ImageEncodingNames = {
            { ImageEncoding::rgba8, "rgba8" },
            { ImageEncoding::rgb8, "rgb8" },
            { ImageEncoding::mono8, "mono8" },
            { ImageEncoding::mono16, "mono16" },
        };
        const AZStd::unordered_map<AZStd::string, ImageEncoding> ImageEncodingFromName = {
            { "rgba8", ImageEncoding::rgba8 },
            { "rgb8", ImageEncoding::rgb8 },
            { "mono8", ImageEncoding::mono8 },
            { "mono16", ImageEncoding::mono16 },
        };

        const AZStd::unordered_map<EncodingConverion, const AZStd::function<void(sensor_msgs::msg::Image&)>> supportedFormatChange = {
            { { ImageEncoding::rgba8, ImageEncoding::rgb8 }, Rgba8ToRgb8 },
        };
    } // namespace

    void EncodingConverion::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EncodingConverion>()
                ->Version(0)
                ->Field("EncodingIn", &EncodingConverion::encodingIn)
                ->Field("EncodingOut", &EncodingConverion::encodingOut);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<EncodingConverion>("Encoding Conversion", "Specifies encoding conversion")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox, &EncodingConverion::encodingIn, "Encoding In", "Encoding of the input image")
                    ->EnumAttribute(ImageEncoding::rgba8, "rgba8")
                    ->EnumAttribute(ImageEncoding::rgb8, "rgb8")
                    ->EnumAttribute(ImageEncoding::mono8, "mono8")
                    ->EnumAttribute(ImageEncoding::mono16, "mono16")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox, &EncodingConverion::encodingOut, "Encoding Out", "Encoding of the output image")
                    ->EnumAttribute(ImageEncoding::rgba8, "rgba8")
                    ->EnumAttribute(ImageEncoding::rgb8, "rgb8")
                    ->EnumAttribute(ImageEncoding::mono8, "mono8")
                    ->EnumAttribute(ImageEncoding::mono16, "mono16");
            }
        }
    }

    void ROS2ImageEncodingConvertComponent::Reflect(AZ::ReflectContext* context)
    {
        EncodingConverion::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2ImageEncodingConvertComponent, AZ::Component>()
                ->Version(0)
                ->Field("Priority", &ROS2ImageEncodingConvertComponent::m_priority)
                ->Field("EncodingConvertData", &ROS2ImageEncodingConvertComponent::m_encodingConvertData);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<ROS2ImageEncodingConvertComponent>(
                      "Image Encoding Conversion Component", "Converts image encoding to a different encoding")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2CameraSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2CameraSensor.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2ImageEncodingConvertComponent::m_priority,
                        "Priority",
                        "Priority of the post processing. The higher the number the later the post processing is applied.")
                    ->Attribute(AZ::Edit::Attributes::Min, CameraPostProcessingRequests::MIN_PRIORITY)
                    ->Attribute(AZ::Edit::Attributes::Max, CameraPostProcessingRequests::MAX_PRIORITY)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2ImageEncodingConvertComponent::m_encodingConvertData,
                        "Encoding Conversion",
                        "Specifies the encoding conversion to apply")
                    ->Attribute(AZ::Edit::Attributes::ChangeValidate, &ROS2ImageEncodingConvertComponent::ValidateEncodingConversion);
            }
        }
    }

    void ROS2ImageEncodingConvertComponent::Activate()
    {
        CameraPostProcessingRequestBus::Handler::BusConnect(GetEntityId());
    }

    void ROS2ImageEncodingConvertComponent::Deactivate()
    {
        CameraPostProcessingRequestBus::Handler::BusDisconnect();
    }

    void ROS2ImageEncodingConvertComponent::ApplyPostProcessing(sensor_msgs::msg::Image& image)
    {
        auto nameIter = ImageEncodingFromName.find(image.encoding.c_str());
        if (nameIter == ImageEncodingFromName.end())
        {
            return;
        }
        ImageEncoding encoding = nameIter->second;
        if (encoding != m_encodingConvertData.encodingIn)
        {
            return;
        }

        auto convertIter = supportedFormatChange.find(m_encodingConvertData);
        if (convertIter == supportedFormatChange.end())
        {
            return;
        }

        convertIter->second(image);
    }

    AZ::u8 ROS2ImageEncodingConvertComponent::GetPriority() const
    {
        return m_priority;
    }

    AZ::Outcome<void, AZStd::string> ROS2ImageEncodingConvertComponent::ValidateEncodingConversion(
        void* newValue, const AZ::Uuid& valueType)
    {
        EncodingConverion* data = reinterpret_cast<EncodingConverion*>(newValue);
        if (supportedFormatChange.find(*data) == supportedFormatChange.end())
        {
            return AZ::Failure(AZStd::string::format(
                "Unsupported encoding change from %s to %s",
                ImageEncodingNames.at(data->encodingIn),
                ImageEncodingNames.at(data->encodingOut)));
        }
        return AZ::Success();
    }

} // namespace ROS2
