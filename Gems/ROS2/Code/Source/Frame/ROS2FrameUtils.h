/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Component/Entity.h>

namespace ROS2 {
    namespace Internal
    {
        AZ::TransformInterface* GetEntityTransformInterface(const AZ::Entity* entity);
    } // namespace Internal
} // namespace ROS2
