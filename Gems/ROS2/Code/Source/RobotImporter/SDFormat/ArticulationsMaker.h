/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/unordered_map.h>
#include <PhysX/ArticulationTypes.h>
#include <urdf_parser/urdf_parser.h>

namespace ROS2::SDFormat
{
    //! Populates the entity with contents of the <inertial> and <joint> tag in the robot description.
    class ArticulationsMaker
    {
    public:
        //! Add zero or one inertial and joints elements to a given entity (depending on link content).
        //! @param link A pointer to a parsed URDF link.
        //! @param entityId A non-active entity which will be populated according to inertial content.
        void AddArticulationLink(urdf::LinkSharedPtr link, AZ::EntityId entityId) const;
    };
} // namespace ROS2::SDFormat
