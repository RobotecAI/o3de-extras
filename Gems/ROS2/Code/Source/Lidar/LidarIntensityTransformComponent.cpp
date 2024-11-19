#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <Lidar/LidarIntensityTransformComponent.h>

namespace ROS2
{
    void LidarIntensityTransformComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void LidarIntensityTransformComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<LidarIntensityTransformComponent, AZ::Component>()
                ->Version(0)
                ->Field("lidarCore", &LidarIntensityTransformComponent::m_logFactor)
                ->Field("messageFormat", &LidarIntensityTransformComponent::m_linearFactor);

            if (auto* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<LidarIntensityTransformComponent>("ROS2 Lidar Intensity transform", "Lidar intensity transform")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LidarIntensityTransformComponent::m_logFactor, "Logarithmic factor", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LidarIntensityTransformComponent::m_linearFactor, "Linear factor", "");
            }
        }
    }

    void LidarIntensityTransformComponent::Activate()
    {
        LidarResultsTransformBus::Handler::BusConnect(GetEntityId());
    }

    void LidarIntensityTransformComponent::Deactivate()
    {
        LidarResultsTransformBus::Handler::BusDisconnect();
    }

    void LidarIntensityTransformComponent::TransformResults(RaycastResults& raycastResults)
    {
        auto intensitySpan = raycastResults.GetFieldSpan<RaycastResultFlags::Intensity>();
        if (!intensitySpan.has_value())
        {
            return;
        }

        for (auto intensityIt = intensitySpan->begin(); intensityIt != intensitySpan->end(); ++intensityIt)
        {
            *intensityIt = TransformIntensity(*intensityIt);
        }
    }
} // namespace ROS2
