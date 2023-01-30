/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2CameraSensorPublisher.h"
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <sensor_msgs/distortion_models.hpp>



namespace ROS2
{
    namespace Internal1
    {

        /// @FormatMappings - contains the mapping from RHI to ROS image encodings. List of supported
        /// ROS image encodings lives in `sensor_msgs/image_encodings.hpp`
        /// We are not including `image_encodings.hpp` since it uses exceptions.
        AZStd::unordered_map<AZ::RHI::Format, const char*> FormatMappings{
            { AZ::RHI::Format::R8G8B8A8_UNORM, "rgba8" },     
            { AZ::RHI::Format::R16G16B16A16_UNORM, "rgba16" },
            { AZ::RHI::Format::R32G32B32A32_FLOAT, "32FC4" }, // Unsuported by RVIZ2
            { AZ::RHI::Format::R8_UNORM, "mono8" },           
            { AZ::RHI::Format::R16_UNORM, "mono16" },
            { AZ::RHI::Format::R32_FLOAT, "32FC1" },
        };

        /// @BitDepth - contains the mapping from RHI to size used in `step` size computation.
        /// It is some equivalent to `bitDepth()` function from `sensor_msgs/image_encodings.hpp`
        AZStd::unordered_map<AZ::RHI::Format, int> BitDepth{
            { AZ::RHI::Format::R8G8B8A8_UNORM, 4 * sizeof(uint8_t) },
            { AZ::RHI::Format::R16G16B16A16_UNORM, 4 * sizeof(uint16_t) },
            { AZ::RHI::Format::R32G32B32A32_FLOAT, 4 * sizeof(float) }, // Unsuported by RVIZ2
            { AZ::RHI::Format::R8_UNORM, sizeof(uint8_t) },
            { AZ::RHI::Format::R16_UNORM, sizeof(uint16_t) },
            { AZ::RHI::Format::R32_FLOAT, sizeof(float) },
        };

    } // namespace Internal1

    void ROS2CameraSensorPublisher::SetupPublisher(
            AZ::Transform cameraPose,
            std_msgs::msg::Header header)
    {
            m_cameraPose = cameraPose;
            m_header = header;
    }

    void ROS2CameraSensorPublisher::Publish(const AZ::RPI::AttachmentReadback::ReadbackResult& result) const
    {
        const AZ::RHI::ImageDescriptor& descriptor = result.m_imageDescriptor;
        const auto format = descriptor.m_format;
        AZ_Assert(Internal1::FormatMappings.contains(format), "Unknown format in result %u", static_cast<uint32_t>(format));
        sensor_msgs::msg::Image message;
        message.encoding = Internal1::FormatMappings.at(format);
        message.width = descriptor.m_size.m_width;
        message.height = descriptor.m_size.m_height;
        message.step = message.width * Internal1::BitDepth.at(format);
        message.data = std::vector<uint8_t>(result.m_dataBuffer->data(), result.m_dataBuffer->data() + result.m_dataBuffer->size());
        message.header = m_header;

        m_publisher->publish(message);
    }
    

} // namespace ROS2
