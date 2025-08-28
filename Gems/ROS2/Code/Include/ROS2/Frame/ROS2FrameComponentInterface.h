/*
* Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <ROS2/Frame/ROS2FrameConfiguration.h>
namespace ROS2
{
    //! Base class to get configration from component
    class ROSFrameInterface
    {
        AZ_RTTI(ROSFrameInterface, ROSFrameInterfaceTypeId);
    public:
        virtual ROS2FrameConfiguration GetConfiguration() const = 0;
    };

}