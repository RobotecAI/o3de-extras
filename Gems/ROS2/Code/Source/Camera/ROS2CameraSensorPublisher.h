/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <ROS2/Sensor/ROS2SensorComponent.h>

#include <AzCore/Component/Component.h>

#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Frame/ROS2Transform.h>
#include <Atom/RPI.Public/Pass/AttachmentReadback.h>

namespace ROS2
{

    //! 
    class ROS2CameraSensorPublisher 
    {
    public:
        ROS2CameraSensorPublisher() {};
        ~ROS2CameraSensorPublisher() {};

        void AddPublisher(std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> publisher){m_publisher = publisher;};

        void SetupPublisher(AZ::Transform cameraPose,
            std_msgs::msg::Header header);

        void Publish(const AZ::RPI::AttachmentReadback::ReadbackResult& result) const;

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> getPublisher() const { return m_publisher; };
        AZ::Transform  getCameraPose() {return m_cameraPose; };
        std_msgs::msg::Header getHeader() const { return m_header; };

        private:
            std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> m_publisher;
            AZ::Transform m_cameraPose;
            std_msgs::msg::Header m_header;

    };
} // namespace ROS2
