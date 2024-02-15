/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AzCore/Component/EntityId.h"
#include "CameraSensorConfiguration.h"
#include <AzCore/Math/Matrix4x4.h>
#include <AzCore/std/containers/array.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Sensor/SensorConfiguration.h>

namespace ROS2
{
    //! Structure containing all information required to create the camera sensor.
    struct CameraSensorDescription
    {
        //! Type of camera channel.
        enum class CameraChannelType
        {
            RGB = 0,
            DEPTH = 1
        };

        //! Constructor to create the description
        //! @param entityId - entityId for the owning sensor component.
        //! @param configuration - configuration structure for the camera, defining its characteristics.
        //! @param sensorConfiguration - generic configuration for this sensor.
        CameraSensorDescription(
            const AZ::EntityId& entityId, const CameraSensorConfiguration& configuration, const SensorConfiguration& sensorConfiguration);

        const CameraSensorConfiguration m_cameraConfiguration; //!< Configuration of the camera.
        const SensorConfiguration m_sensorConfiguration; //!< Generic sensor configuration.
        const AZ::EntityId m_entityId;

        const AZ::Matrix4x4 m_viewToClipMatrix; //!< Camera view to clip space transform matrix; derived from other parameters.
        const AZ::Matrix3x3 m_cameraIntrinsics; //!< Camera intrinsics; derived from other parameters.

    private:
        void ValidateParameters() const;
    };
} // namespace ROS2
