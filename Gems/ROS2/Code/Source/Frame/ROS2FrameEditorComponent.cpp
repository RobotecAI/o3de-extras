/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2FrameEditorComponent.h"
#include "ROS2FrameSystemBus.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/EntityUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzToolsFramework/UI/PropertyEditor/PropertyEditorAPI.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameEditorComponentBus.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2NamesBus.h>
#include <AzCore/Component/TransformBus.h>
namespace ROS2
{
    ROS2FrameEditorComponent::ROS2FrameEditorComponent(const ROS2FrameConfiguration ros2FrameConfiguration)
    {
        m_configuration = ros2FrameConfiguration;
    }

    void ROS2FrameEditorComponent::Init()
    {

    }

    void ROS2FrameEditorComponent::Activate()
    {
        AZ_Printf("BuildGameEntity", "ROS2FrameEditorComponent::Activate for entity %s", this->GetEntityId().ToString().c_str());
        ROS2FrameEditorComponentBus::Handler::BusConnect(GetEntityId());
        AZ::EntityBus::Handler::BusConnect(GetEntityId());
        if (auto* frameSystemInterface = ROS2FrameSystemInterface::Get())
        {
            frameSystemInterface->RegisterFrame(GetEntityId());
        }
    }

    void ROS2FrameEditorComponent::Deactivate()
    {
        AZ_Printf("BuildGameEntity", "ROS2FrameEditorComponent::Deactivate for entity %s", this->GetEntityId().ToString().c_str());
        if (auto* frameSystemInterface = ROS2FrameSystemInterface::Get())
        {
            frameSystemInterface->UnregisterFrame(GetEntityId());
        }
        AZ::EntityBus::Handler::BusDisconnect();
        ROS2FrameEditorComponentBus::Handler::BusDisconnect();
    }

    AZStd::string ROS2FrameEditorComponent::GetGlobalFrameName() const
    {
        AZStd::string namespacedFrameName;
        ROS2NamesRequestBus::BroadcastResult(
            namespacedFrameName, &ROS2NamesRequests::GetNamespacedName, GetNamespace(), AZStd::string("odom"));

        return namespacedFrameName;
    }

    bool ROS2FrameEditorComponent::IsTopLevel() const
    {
        return ROS2FrameSystemInterface::Get()->IsTopLevel(GetEntityId());
    }

    AZStd::string ROS2FrameEditorComponent::GetNamespacedFrameID() const
    {
        AZStd::string namespacedFrameID;
        ROS2NamesRequestBus::BroadcastResult(
            namespacedFrameID, &ROS2NamesRequests::GetNamespacedName, GetNamespace(), m_configuration.m_frameName);
        return namespacedFrameID;
    }

    AZStd::string ROS2FrameEditorComponent::GetNamespace() const
    {
        return "";//m_configuration.m_namespaceConfiguration.GetNamespace();
    }

    void ROS2FrameEditorComponent::UpdateNamespace(const AZStd::string& parentNamespace)
    {
        AZ_Printf("ROS2FrameEditorComponent", "Updating namespace for entity %s", GetEntity()->GetName().c_str());

        m_effectiveNamespace = ROS2FrameSystemInterface::Get()->GetNamespace(m_configuration, GetEntityId());
        m_fullName = ROS2FrameSystemInterface::Get()->GetFrameName(m_configuration, GetEntityId());

        AzToolsFramework::PropertyEditorEntityChangeNotificationBus::Event(
            GetEntityId(), &AzToolsFramework::PropertyEditorEntityChangeNotificationBus::Events::OnEntityComponentPropertyChanged, GetId());

    }

    AZ::Name ROS2FrameEditorComponent::GetNamespacedJointName() const
    {
        AZStd::string namespacedJointName;
        ROS2NamesRequestBus::BroadcastResult(
            namespacedJointName, &ROS2NamesRequests::GetNamespacedName, GetNamespace(), m_configuration.m_jointName);
        return AZ::Name(namespacedJointName.c_str());
    }

    void ROS2FrameEditorComponent::SetJointName(const AZStd::string& jointName)
    {
        m_configuration.m_jointName = jointName;
    }

    void ROS2FrameEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameEditorComponent, AzToolsFramework::Components::EditorComponentBase>()->Version(1)->Field(
                "ROS2FrameConfiguration", &ROS2FrameEditorComponent::m_configuration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2FrameEditorComponent>("ROS2 Frame", "[ROS2 Frame component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Level"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2Frame.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2Frame.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2FrameEditorComponent::m_configuration,
                        "ROS 2 Frame Configuration",
                        "Configuration of ROS 2 reference frame")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &ROS2FrameEditorComponent::OnFrameConfigurationChange)
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues)
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Info")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->UIElement(AZ::Edit::UIHandlers::Label, "Effective namespace", "")
                    ->Attribute(AZ::Edit::Attributes::ValueText, &ROS2FrameEditorComponent::m_effectiveNamespace)
                    ->UIElement(AZ::Edit::UIHandlers::Label, "Full name", "")
                    ->Attribute(AZ::Edit::Attributes::ValueText, &ROS2FrameEditorComponent::m_fullName);

            }
        }
    }

    AZ::EntityId ROS2FrameEditorComponent::GetFrameParent() const
    {
        return ROS2FrameSystemInterface::Get()->GetParentEntityId(GetEntityId());
    }

    AZStd::set<AZ::EntityId> ROS2FrameEditorComponent::GetFrameChildren() const
    {
        return ROS2FrameSystemInterface::Get()->GetChildrenEntityId(GetEntityId());
    }

    AZ::Crc32 ROS2FrameEditorComponent::OnFrameConfigurationChange()
    {
        m_effectiveNamespace = ROS2FrameSystemInterface::Get()->GetNamespace(m_configuration, GetEntityId());
        m_fullName = ROS2FrameSystemInterface::Get()->GetFrameName(m_configuration, GetEntityId());
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    void ROS2FrameEditorComponent::OnEntityNameChanged(const AZStd::string& name)
    {
        OnFrameConfigurationChange();
    }

    void ROS2FrameEditorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2FrameEditorComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2FrameEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }
    void test(AZ::EntityId id)
    {
        // Get the transform component
        // get entity
        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, id);
        if (!entity)
        {
            return ;
        }

        AZ_Printf("TESTFOOO", "%s", entity->GetName().c_str());
        auto* transform = entity->GetTransform();
        if (!transform)
        {
            return;
        }
        AZ::EntityId parentId = transform->GetParentId();
        if (!parentId.IsValid())
        {
            return ;
        }
        return test(parentId);
    }


    void ROS2FrameEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {

        AZ_Printf("BuildGameEntity", "ROS2FrameEditorComponent::BuildGameEntity for entity %s", this->GetEntityId().ToString().c_str());
        test(GetEntityId());
        const auto nameSpace  = ROS2FrameSystemInterface::Get()->GetNamespace(m_configuration, GetEntityId());
        const auto frameName = m_configuration.m_frameName;
        const auto jointName = m_configuration.m_jointName;
        AZ_Printf("ROS2FrameEditorComponent", "Creating ROS2FrameComponent on entity %s, fn: %s, jn: %s, ns: %s", gameEntity->GetName().c_str(), frameName.c_str(), jointName.c_str(), nameSpace.c_str());
        const bool publishTransform = m_configuration.m_publishTransform;
        gameEntity->CreateComponent<ROS2FrameComponent>(frameName, jointName, nameSpace, publishTransform, true);
    }

    ROS2FrameConfiguration ROS2FrameEditorComponent::GetConfiguration() const
    {
        return m_configuration;
    }

} // namespace ROS2
