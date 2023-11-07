#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>

#pragma once

class Ros2ImGui
{
    public: 
    Ros2ImGui();
    ~Ros2ImGui();
    void Draw(AZ::Entity * entity);

    private:
    // get list of all components added to the entity (the one where this component is attached to)
    AZStd::vector<AZ::Component *> GetComponents(AZ::EntityId entityId);
    void FilterChildComponents(AZStd::vector<AZ::Component *> & components);
    AZ::Entity * GetParentEntity(AZ::Entity * entity);
    AZ::TransformInterface* GetEntityTransformInterface(const AZ::Entity* entity);
};
