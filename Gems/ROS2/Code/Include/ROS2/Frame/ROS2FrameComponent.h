/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Serialization/Json/BaseJsonSerializer.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <AzFramework/Components/TransformComponent.h>
#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>
#include <ROS2/Frame/ROS2Transform.h>
#include <ROS2/ROS2TypeIds.h>
#include <ROS2/Frame/ROS2FrameComponentInterface.h>

namespace ROS2
{


    //! This component marks an interesting reference frame for ROS2 ecosystem.
    //! It serves as sensor data frame of reference and is responsible, through ROS2Transform, for publishing
    //! ros2 static and dynamic transforms (/tf_static, /tf). It also facilitates namespace handling.
    //! An entity can only have a single ROS2Frame on each level. Many ROS2 Components require this component.
    //! @note A robot should have this component on every level of entity hierarchy (for each joint, fixed or dynamic)
    class ROS2FrameComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public ROSFrameInterface
    {
        friend class JsonFrameComponentConfigSerializer;

    public:
        AZ_COMPONENT(ROS2FrameComponent, ROS2FrameComponentTypeId, ROSFrameInterface);

        ROS2FrameComponent();

        ROS2FrameComponent(const AZStd::string& targetFrame,
            const AZStd::string& jointName = "",
            const AZStd::string nameSpace = "",
            bool publishTransform = true,
            bool isDynamic= true);

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

        static void Reflect(AZ::ReflectContext* context);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        AZStd::string GetSourceFrame() const
        {
            return m_sourceFrame.value_or("");
        }

        bool IsDynamic() const
        {
            return m_isDynamic;
        }

        //! Setters for game component configuration
        //! @note The methods should be called only when the component is deactivated (e.g. during preparation of spawning the entity)
        //! Changing configuration on active component is not forbidden, but you should be aware that some changes may cause
        //! unexpected behavior (e.g. changing source or target static frame will republish the static transform - causing jumps in TF tree)

        void SetFrameID(const AZStd::string& targetFrame)
        {
            AZ_Warning("ROS2FrameComponent", this->GetEntity()->GetState() != AZ::Entity::State::Active,  "Changing source frame on active ROS2FrameComponent may cause unexpected behavior.");
            m_targetFrame = targetFrame;
        }
        void SetIsDynamic(bool isDynamic)
        {
            AZ_Warning("ROS2FrameComponent", this->GetEntity()->GetState() != AZ::Entity::State::Active,  "Changing source frame on active ROS2FrameComponent may cause unexpected behavior.");
            m_isDynamic = isDynamic;
        }

        const AZStd::string& GetJointName() const
        {
            return m_jointName;
        }

        const AZStd::string GetNamespacedJointName() const
        {
            if (m_namespace.empty())
            {
                return m_jointName;
            }
            return AZStd::string::format("%s/%s", m_namespace.c_str(), m_jointName.c_str());
        }

        const AZStd::string& GetFrameID() const
        {
            return m_targetFrame;
        }

        const AZStd::string GetNamespacedFrameID() const
        {
            if (m_namespace.empty())
            {
                return m_targetFrame;
            }
            return AZStd::string::format("%s/%s", m_namespace.c_str(), m_targetFrame.c_str());
        }

        const AZStd::string& GetNamespace() const
        {
            return m_namespace;
        }

        ROS2FrameConfiguration GetConfiguration() const override
        {
            return ROS2FrameConfiguration();
        }
    private:
        AZ::Transform GetFrameTransform() const;
        const ROS2FrameComponent* GetParentROS2FrameComponent() const;
        //////////////////////////////////////////////////////////////////////////
        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        //////////////////////////////////////////////////////////////////////////

        AZStd::optional<AZ::EntityId> m_parentFrame; //!< Cached parent entity with ROS2FrameComponent, if any
        AZStd::optional<AZStd::string> m_sourceFrame; //!< If not set, the source frame is assumed to be the parent frame in the TF tree
        AZStd::unique_ptr<ROS2Transform> m_ros2Transform;
        AZStd::string m_targetFrame; //! frame name (without namespace)
        AZStd::string m_jointName; //! joint name (without namespace)
        AZStd::string m_namespace; //! namespace for this frame and joint
        bool m_publishTransform = true;
        bool m_isDynamic = true;
    };
} // namespace ROS2
