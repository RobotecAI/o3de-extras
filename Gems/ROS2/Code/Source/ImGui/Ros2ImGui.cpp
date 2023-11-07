#include "Ros2ImGui.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityUtils.h>
#include <AzFramework/Components/TransformComponent.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Sensor/SensorConfiguration.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <ROS2/Sensor/Events/PhysicsBasedSource.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>

#include <imgui/imgui.h>
#include <string>
#include <algorithm>

auto controlSensorComponent = [](auto* sensorComponent, ImGuiContext& imguiContext, const std::string& uniqueId) {
    std::string disableButtonLabel = "Disable Component##" + uniqueId;
    if (ImGui::Button(disableButtonLabel.c_str())) {
        sensorComponent->Deactivate();
    }
    ImGui::SameLine();
    std::string enableButtonLabel = "Enable Component##" + uniqueId;
    if (ImGui::Button(enableButtonLabel.c_str())) {
        sensorComponent->Activate();
    }
};


Ros2ImGui::Ros2ImGui()
{
}

Ros2ImGui::~Ros2ImGui()
{
}

void Ros2ImGui::Draw(AZ::Entity * entity)
{
  if (!entity) {
    AZ_Error("Draw", false, "Invalid entity!");
    return;
  }

  auto childComponents = GetComponents(entity->GetId());
//   entity->FindComponents()
  if (childComponents.empty()) {
    AZ_Error("Draw", false, "Invalid child components!");
    return;
  }
  FilterChildComponents(childComponents);
  ImGui::Begin("Sensors");
  ImGui::Text("Components of entity: %s", entity->GetName().c_str());
  for (auto * component : childComponents) {
    ImGui::Text("Component: %s", component->RTTI_GetTypeName());

    auto * sensorComponentTickBased =
      azrtti_cast<ROS2::ROS2SensorComponentBase<ROS2::TickBasedSource> *>(component);
    auto * sensorComponentPhysicsBased =
      azrtti_cast<ROS2::ROS2SensorComponentBase<ROS2::PhysicsBasedSource> *>(component);

    // Generate a unique identifier for the ImGui button labels
    const std::string uniqueId = std::to_string(reinterpret_cast<uintptr_t>(component));

    if (sensorComponentTickBased) {
      controlSensorComponent(sensorComponentTickBased, *ImGui::GetCurrentContext(), uniqueId);
    } else if (sensorComponentPhysicsBased) {
      controlSensorComponent(sensorComponentPhysicsBased, *ImGui::GetCurrentContext(), uniqueId);
    }

    ImGui::Text("----");
  }
  ImGui::End();
}

// Private

AZStd::vector<AZ::Component *> Ros2ImGui::GetComponents(AZ::EntityId entityId)
{
  AZStd::vector<AZ::Component *> components;
  AZ::Entity * entity = nullptr;
  AZ::ComponentApplicationBus::BroadcastResult(
    entity,
    &AZ::ComponentApplicationRequests::FindEntity,
    entityId);
  if (!entity) {
    AZ_Error("GetComponents", false, "Invalid entity!");
    return components;
  }

  AZ::Entity::ComponentArrayType entityComponents = entity->GetComponents();
  for (auto * component : entityComponents) {
    components.push_back(component);
  }
  return components;
}


AZ::TransformInterface * Ros2ImGui::GetEntityTransformInterface(const AZ::Entity * entity)
{
  if (!entity) {
    AZ_Error("GetEntityTransformInterface", false, "Invalid entity!");
    return nullptr;
  }

  auto * interface =
    ::ROS2::Utils::GetGameOrEditorComponent<AzFramework::TransformComponent>(entity);

  return interface;
}

void Ros2ImGui::FilterChildComponents(AZStd::vector<AZ::Component *> & components)
{
  // remove the components that typeName does not end with "SensorComponent"
  components.erase(
    std::remove_if(
      components.begin(),
      components.end(),
      [](AZ::Component * component) {
        std::string typeName = component->RTTI_GetTypeName();         // Convert to std::string
        std::string suffix = "SensorComponent";
        // Check if the type name ends with "SensorComponent"
        return !(typeName.rfind(suffix) == typeName.size() - suffix.size());
      }),
    components.end());

  // Remove also the ImGuiComponent
  components.erase(
    std::remove_if(
      components.begin(),
      components.end(),
      [](AZ::Component * component) {
        std::string typeName = component->RTTI_GetTypeName();         // Convert to std::string
        std::string suffix = "ImGuiComponent";
        // Check if the type name ends with "ImGuiComponent"
        return typeName.rfind(suffix) == typeName.size() - suffix.size();
      }),
    components.end());
}
