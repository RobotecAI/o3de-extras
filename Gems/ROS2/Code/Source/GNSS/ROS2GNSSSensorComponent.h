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
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <ROS2/GNSS/GNSSPostProcessingRequestBus.h>
#include "GNSSPostProcessing.h"

namespace ROS2
{
    //! Global Navigation Satellite Systems (GNSS) sensor component class
    //! It provides NavSatFix data of sensor's position in GNSS frame which is defined by GNSS origin offset
    //! Offset is provided as latitude [deg], longitude [deg], altitude [m] of o3de global frame
    //! It is assumed that o3de global frame overlaps with ENU coordinate system
    class ROS2GNSSSensorComponent
        : public ROS2SensorComponentBase<TickBasedSource>
        , public GNSSPostProcessingRequestBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2GNSSSensorComponent, "{55B4A299-7FA3-496A-88F0-764C75B0E9A7}", SensorBaseType);
        ROS2GNSSSensorComponent();
        ROS2GNSSSensorComponent(const SensorConfiguration& sensorConfiguration);
        ~ROS2GNSSSensorComponent() = default;
        static void Reflect(AZ::ReflectContext* context);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////
        //! Returns true if the sensor has a fix, false otherwise.
        bool GetFixState();
        void SetFixState(bool isFix);
        void ToggleFixLoss(); 

    private:
        ///! Requests gnss message publication.
        void FrequencyTick();
        void UpdateGnssMessage();

        //! Changes the message

        //! Returns current entity position.
        //! @return Current entity position.
        [[nodiscard]] AZ::Transform GetCurrentPose() const;

        //! GNSSPostProcessingRequestBus::Handler overrides
        void ApplyPostProcessing(sensor_msgs::msg::NavSatFix& gnss) override;

        AZ::Crc32 OnPostProcessingChanged() const;
        bool IsPostProcessingApplied() const;

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> m_gnssPublisher;
        sensor_msgs::msg::NavSatFix m_gnssMsg;
        bool m_isFix = true;
        bool m_applyPostProcessing = false;
        NoiseType m_noiseType;
        AZStd::unique_ptr<GNSSPostProcessing> m_gnssPostProcessing;
    };

} // namespace ROS2
