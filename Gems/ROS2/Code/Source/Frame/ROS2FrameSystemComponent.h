/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AzCore/Component/TickBus.h"
#include "ROS2FrameSystemBus.h"

#include <AzCore/Component/Component.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/std/containers/map.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>

namespace ROS2
{

    //! Component which manages the frame entities and their hierarchy.
    //! It is responsible for updating the namespaces of the frame entities and their children.
    //! It also notifies the ROS2FrameEditorComponent about changes in the tree.
    //! Used to register, unregister, track the frame entities in the level entity tree.
    class ROS2FrameSystemComponent
        : public AZ::Component
        , protected ROS2FrameSystemInterface::Registrar
        , protected AzToolsFramework::EntitySelectionEvents::Bus::MultiHandler
        , protected AZ::TransformNotificationBus::MultiHandler
    {
    public:
        AZ_COMPONENT(ROS2FrameSystemComponent, "{360c4b45-ac02-42d2-9e1a-1d77eb22a054}");
        static void Reflect(AZ::ReflectContext* context);

        // AZ::Component overrides.
        void Activate() override;
        void Deactivate() override;

        ROS2FrameSystemComponent();

        ~ROS2FrameSystemComponent();

    private:
        // ROS2FrameSystemInterface::Registrar overrides.
        void RegisterFrame(const AZ::EntityId& frameEntityId) override;
        void UnregisterFrame(const AZ::EntityId& frameEntityId) override;
        bool IsTopLevel(const AZ::EntityId& frameEntityId) const override;
        AZ::EntityId GetParentEntityId(const AZ::EntityId& frameEntityId) const override;
        AZStd::set<AZ::EntityId> GetChildrenEntityId(const AZ::EntityId& frameEntityId) const override;
        AZStd::string GetFrameName(const ROS2FrameConfiguration& configuration, AZ::EntityId entity) const override;
        AZStd::string GetJointName(const ROS2FrameConfiguration& configuration, AZ::EntityId entity) const override;
        AZStd::string GetNamespace(const ROS2FrameConfiguration& configuration, AZ::EntityId entity) const override;

        // EntitySelectionEvents::Bus::MultiHandler overrides.
        void OnSelected() override;
        //! AZ::TransformNotificationBus::MultiHandler overrides.
        void OnParentChanged(AZ::EntityId oldParent, AZ::EntityId newParent) override;


        AZStd::set<AZ::EntityId> m_registeredEntities; //!< Set of all registered frame entities.
         //! Find the path from the frameEntity to the frame parent of that entity.
        //! This path will include the frameEntity and the frame parent.
        //! If there is no frame parent, path to the root entity (included) will be returned.
        //! @param frameEntityId frame to find the path to the parent.
        //! @return vector of entityIds which represent the path to the parent. frameEntityId is first, parent is last.
        AZStd::vector<AZ::EntityId> FindFrameParentPath(AZ::EntityId frameEntityId);

        AZ::TransformInterface* GetEntityTransformInterface(const AZ::Entity* entity);

        //! Updates the namespaces of all children of the frameEntity.
        //! @param frameEntity frame to be updated.
        //! @param parentNamespace namespace of the parent frame. Empty if no parent is present.
        //! @param isActive boolean value describing if the frameEntity is currently active.
        void UpdateNamespaces(AZ::EntityId frameEntity, AZStd::string parentNamespace = "", bool isActive = true);

        //! Updates the namespaces of all children of the frameEntity.
        //! @param frameEntity frame to be updated.
        //! @param frameParentEntity entityId of the parent frame.
        //! @param isActive boolean value describing if the frameEntity is currently active.
        void UpdateNamespaces(AZ::EntityId frameEntity, AZ::EntityId frameParentEntity, bool isActive = true);

        void MoveFrameDetach(const AZ::EntityId& frameEntityId, const AZStd::set<AZ::EntityId>& newPathToParentFrameSet);
        void MoveFrameAttach(
            const AZ::EntityId& frameEntityId, const AZ::EntityId& newFrameParent, const AZStd::vector<AZ::EntityId>& newPathToParentFrame);


    };
} // namespace ROS2
