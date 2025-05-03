/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Entity.h>

namespace ROS2Controllers::JointUtils
{
    //! Checks whether the entity has a component of the given type
    //! @param entity pointer to entity
    //! @param typeId type of the component
    //! @returns true if entity has component with given type
    bool HasComponentOfType(const AZ::Entity* entity, const AZ::Uuid& typeId);

    //! Check if the entity has any of the non fixed joint or articulation components.
    //! @param entity Entity to check.
    //! @return True if the entity has any of the joint or articulation components.
    bool HasNonFixedJoints(const AZ::Entity* entity);
} // namespace ROS2Controllers::JointUtils
