/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "Spawner/ROS2SpawnerComponentController.h"
#include <AzCore/Component/Component.h>
#include <AzFramework/Components/ComponentAdapter.h>

namespace ROS2
{
    using ROS2SpawnerComponentBase = AzFramework::Components::ComponentAdapter<ROS2SpawnerComponentController, ROS2SpawnerComponentConfig>;
    //! Deprecated robot spawner, kept only so that existing levels and prefabs still load.
    //! Spawning was removed together with the gazebo_msgs package; use the SimulationInterfaces Gem instead.
    class ROS2SpawnerComponent : public ROS2SpawnerComponentBase
    {
    public:
        AZ_COMPONENT(ROS2SpawnerComponent, "{8ea91880-0067-11ee-be56-0242ac120002}", AZ::Component);

        // ROS2SpawnerComponentBase interface implementation.
        ROS2SpawnerComponent() = default;
        ROS2SpawnerComponent(const ROS2SpawnerComponentConfig& properties);
        ~ROS2SpawnerComponent() = default;

        static void Reflect(AZ::ReflectContext* context);
    };
} // namespace ROS2
