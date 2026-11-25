/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SpawnerComponent.h"
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Spawner/SpawnerBus.h>
#include <ROS2/Spawner/SpawnerBusHandler.h>

namespace ROS2
{

    ROS2SpawnerComponent::ROS2SpawnerComponent(const ROS2SpawnerComponentConfig& properties)
        : ROS2SpawnerComponentBase(properties)
    {
    }

    void ROS2SpawnerComponent::Reflect(AZ::ReflectContext* context)
    {
        ROS2SpawnerComponentBase::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2SpawnerComponent, ROS2SpawnerComponentBase>()->Version(1);
        }

        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<SpawnerNotificationBus>("ROS2SpawnerNotificationBus")->Handler<SpawnerNotificationsBusHandler>();
        }
    }
} // namespace ROS2
