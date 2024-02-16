/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Transform.h>
#include <std_msgs/msg/header.hpp>

namespace ROS2
{
    //! Interface for camera sensor implementation
    //! It is used to request image message publication from rendering pipeline
    //! @param cameraPose - current camera pose from which the rendering should take place
    //! @param header - header with filled message information (frame, timestamp)
    class CameraSensor
    {
    public:
        CameraSensor() = default;
        virtual ~CameraSensor() = default;

        //! Publish Image Message frame from rendering pipeline
        //! @param cameraPose - current camera pose from which the rendering should take place
        //! @param header - header with filled message information (frame, timestamp, seq)
        virtual void RequestMessagePublication(const AZ::Transform& cameraPose, const std_msgs::msg::Header& header) = 0;
    };

    //! Interface allows to obrain / change internal ros2 camera sensor implementation.
    class CameraSensorRequests
    {
    public:
        CameraSensorRequests() = default;
        virtual ~CameraSensorRequests() = default;

        using CameraSensorPtr = AZStd::shared_ptr<CameraSensor>;

        //! Returns the camera sensor implementation.
        [[nodiscard]] virtual CameraSensorPtr GetCameraSensor() = 0;

        //! Changes the camera sensor implementation.
        //! @param cameraSensor - new camera sensor implementation.
        virtual void SetCameraSensor(CameraSensorPtr cameraSensor) = 0;
    };

    class CameraSensorBusTraits : public AZ::EBusTraits
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
    };

    using CameraSensorRequestBus = AZ::EBus<CameraSensorRequests, CameraSensorBusTraits>;
} // namespace ROS2
