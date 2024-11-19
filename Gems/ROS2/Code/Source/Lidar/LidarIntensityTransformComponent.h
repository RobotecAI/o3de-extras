#pragma once

#include <AzCore/Component/Component.h>
#include <Lidar/LidarResultsTransformBus.h>
#include <cmath>

namespace ROS2 {

class LidarIntensityTransformComponent
    : public AZ::Component
    , LidarResultsTransformBus::Handler
    {
public:
    AZ_COMPONENT(LidarIntensityTransformComponent, "{5ca690a8-3e8e-41a2-8ac3-3eb59e759bfb}", AZ::Component);
    LidarIntensityTransformComponent() = default;
    ~LidarIntensityTransformComponent() = default;
    static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
    static void Reflect(AZ::ReflectContext* context);
    //////////////////////////////////////////////////////////////////////////
    // Component overrides
    void Activate() override;
    void Deactivate() override;
    //////////////////////////////////////////////////////////////////////////

private:
    //////////////////////////////////////////////////////////////////////////

    [[nodiscard]] inline float TransformIntensity(float intensity) const
    {
        return -std::log(1.0f - intensity / 255.0f) * m_logFactor + intensity * m_linearFactor;
    }
    void TransformResults(RaycastResults& raycastResults) override;

    float m_logFactor{ 500.0f }, m_linearFactor{ 2.0f };
};

} // ROS2
