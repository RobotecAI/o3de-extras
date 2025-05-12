/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Entity.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <AzCore/std/utility/pair.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <ROS2/Communication/TopicConfiguration.h>

namespace ROS2
{
    //! Editor Component responsible for a hierarchical system of joints such as robotic arm with Articulations or Hinge Joints.
    class JointsStateFollowerEditorComponent : public AzToolsFramework::Components::EditorComponentBase
    {
    public:
        JointsStateFollowerEditorComponent();
        ~JointsStateFollowerEditorComponent() = default;
        AZ_EDITOR_COMPONENT(JointsStateFollowerEditorComponent, "{398e1fb4-47cb-4d3d-827b-a46ad5beb4a3}");

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);

        // AzToolsFramework::Components::EditorComponentBase overrides
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        AZ::Crc32 FindAllJoints();

        TopicConfiguration m_topicConfiguration; //!< Configuration of the subscribed topic.
        AZStd::vector<AZStd::string> m_jointNames; //!< Ordered list of joint names that can be modified via subscriber
    };
} // namespace ROS2
