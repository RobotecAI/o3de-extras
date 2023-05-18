/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2FrameUtils.h"

namespace ROS2 {
    namespace Internal
    {
        AZ::TransformInterface* GetEntityTransformInterface(const AZ::Entity* entity)
        {
            if (!entity)
            {
                AZ_Error("GetEntityTransformInterface", false, "Invalid entity!");
                return nullptr;
            }

            auto* interface = Utils::GetGameOrEditorComponent<AzFramework::TransformComponent>(entity);

            return interface;
        }
    } // namespace Internal
} // namespace ROS2
