#include <AzCore/Serialization/SerializeContext.h>

#include "ImGuiComponent.h"

#include <imgui/imgui.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Component/Entity.h>

namespace ROS2::ImGui
{

void ImGuiComponent::Reflect(AZ::ReflectContext * context)
{
  if (auto * serializeContext = azrtti_cast<AZ::SerializeContext *>(context)) {
    serializeContext->Class<ImGuiComponent, AZ::Component>()
    ->Version(0)
    ;
    if (auto * editContext = serializeContext->GetEditContext()) {
      editContext->Class<ImGuiComponent>("ROS2 ImGui", "ImGui for ROS2")
      ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
      ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
      ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
      ;
    }
  }
}

ImGuiComponent::ImGuiComponent() = default;

void ImGuiComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType & provided)
{
  provided.push_back(AZ_CRC_CE("ImGuiService"));
}

void ImGuiComponent::GetIncompatibleServices(
  AZ::ComponentDescriptor::DependencyArrayType & incompatible)
{
  incompatible.push_back(AZ_CRC_CE("ImGuiService"));
}

void ImGuiComponent::GetRequiredServices(
  [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType & required)
{
}

void ImGuiComponent::GetDependentServices(
  [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType & dependent)
{
}

void ImGuiComponent::Activate()
{
  ::ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();
  imGui = new Ros2ImGui();
}

void ImGuiComponent::Deactivate()
{
  ::ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
  delete imGui;
}

void ImGuiComponent::OnImGuiUpdate()
{
  imGui->Draw(GetEntity());
}

}  // namespace ROS2::ImGui