/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>

namespace ROS2::SegmentationUtils
{
    [[nodiscard]] uint8_t FetchClassIdForEntity(AZ::EntityId entityId);
} // namespace ROS2::SegmentationUtils
