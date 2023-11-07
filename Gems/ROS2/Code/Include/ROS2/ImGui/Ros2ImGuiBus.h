#pragma once 

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ROS2::ImGui
{
    class Ros2ImGuiRequests 
    { 
        AZ_RTTI(Ros2ImGuiRequests, "{07BB547B-90C8-4F23-B56A-69B23EF40DD2}");
        virtual ~Ros2ImGuiRequests() = default;

    };

    class Ros2ImGuiBusTraits : public AZ::EBusTraits
    {
    public:
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
    };

    using Ros2ImGuiInterface = AZ::Interface<Ros2ImGuiRequests>;
    using Ros2ImGuiRequestBus = AZ::EBus<Ros2ImGuiRequests, Ros2ImGuiBusTraits>;
    
} // namespace ROS2::ImGui