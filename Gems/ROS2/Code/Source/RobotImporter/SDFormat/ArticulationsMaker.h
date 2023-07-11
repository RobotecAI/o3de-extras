/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/EntityId.h>

#include <sdf/Joint.hh>
#include <sdf/Link.hh>

namespace ROS2::SDFormat
{
    //! Populates the entity with contents of the <inertial> and <joint> tag in the robot description.
    class ArticulationsMaker
    {
    public:
        //! Add zero or one inertial and joints elements to a given entity (depending on link content).
        //! @param link A pointer to a parsed SDFormat link.
        //! @param parentJoint A pointer to a parent joint of a given link in parsed SDFormat data.
        //! @param entityId A non-active entity which will be populated according to inertial content.
        void AddArticulationLink(const sdf::Link* link, const sdf::Joint* parentJoint, AZ::EntityId entityId) const;
    };
} // namespace ROS2::SDFormat
