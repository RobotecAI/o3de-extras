/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <ROS2/Utilities/ROS2Tickers.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace ROS2
{

    //! An IMU (Inertial Measurement Unit) sensor Component.
    //! IMUs typically include gyroscopes, accelerometers and magnetometers. This component encapsulates data
    //! acquisition and its publishing to ROS2 ecosystem. IMU Component requires ROS2FrameComponent.
    class ROS2ImuSensorComponent : public ROS2SensorComponent, public Tickers::SimulatedBodyEventTicker
    {
    public:
        AZ_COMPONENT(ROS2ImuSensorComponent, "{502A955E-7742-4E23-AD77-5E4063739DCA}", ROS2SensorComponent);
        ROS2ImuSensorComponent();
        ~ROS2ImuSensorComponent() = default;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        //////////////////////////////////////////////////////////////////////////
        // SimulatedBodyEventTicker overrides
        void OnEventTick(float deltaTime) override;
        //////////////////////////////////////////////////////////////////////////

        AZ::Vector3 m_previousLinearVelocity = AZ::Vector3::CreateZero();
        AZ::Vector3 m_acceleration = AZ::Vector3::CreateZero();
        int m_filterSize = 10;
        AZStd::deque<AZ::Vector3> m_filter;

        AzPhysics::SimulatedBodyHandle m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        double m_time = 0;

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> m_imuPublisher;
        sensor_msgs::msg::Imu m_imuMsg;
    };
} // namespace ROS2
