/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/EBus/Policies.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/std/containers/set.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>

namespace ROS2
{
    class ROS2FrameSystemRequests
    {
    public:
        AZ_RTTI(ROS2FrameSystemRequests, "{24fe4584-0499-4a37-bc1a-00ca04bd22f5}");

        //! Registers the ROS2FrameEditorComponent into the ROS2FrameSystemComponent. All ROS2FrameEditorComponents should
        //! register using this function during activation.
        //! This is used to properly handle the entity tree and namespaces, for the ROS2FrameEditorComponents.
        //! @param frameEntityId entityId containing the frame to register.
        virtual void RegisterFrame(const AZ::EntityId& frameEntityId) = 0;

        //! Unregister the ROS2FrameEditorComponent into the system. All ROS2FrameEditorComponents should
        //! unregister using this function during deactivation.
        //! @param frameEntityId entityId containing the frame to unregister.
        virtual void UnregisterFrame(const AZ::EntityId& frameEntityId) = 0;

        //! Check if the frame is the highest frame in the entity tree.
        //! @param frameEntityId entityId of the frame to check.
        //! @return boolean value of the check. True for top level.
        virtual bool IsTopLevel(const AZ::EntityId& frameEntityId) const = 0;

        //! Find the parent frame of the entity.
        //! @param frameEntityId entityId of the frame to check.
        //! @return entityId of the parent frame or an invalid entityId if the frame is top level.
        virtual AZ::EntityId GetParentEntityId(const AZ::EntityId& frameEntityId) const = 0;

        //! Find all frame children of the frame.
        //! @param frameEntityId entityId of the frame to check.
        //! @return set of all entityIds of children. Empty if no children or the frameEntityId is invalid.
        virtual AZStd::set<AZ::EntityId> GetChildrenEntityId(const AZ::EntityId& frameEntityId) const = 0;

        //! Resolves the ROS 2 frame name based on configuration and entity ID
        virtual AZStd::string GetFrameName(const ROS2FrameConfiguration& configuration, AZ::EntityId entity) const = 0;

        //! Resolves the ROS 2 namespace based on configuration and entity ID
        virtual AZStd::string GetJointName(const ROS2FrameConfiguration& configuration, AZ::EntityId entity) const = 0;

        //! Resolves the ROS 2 namespace based on configuration and entity ID
        virtual AZStd::string GetNamespace(const ROS2FrameConfiguration& configuration, AZ::EntityId entity) const = 0;

    };

    class ROS2FrameSystemBusTraits : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
    };

    using ROS2FrameSystemInterface = AZ::Interface<ROS2FrameSystemRequests>;
    using ROS2FrameSystemBus = AZ::EBus<ROS2FrameSystemRequests>;
} // namespace ROS2
