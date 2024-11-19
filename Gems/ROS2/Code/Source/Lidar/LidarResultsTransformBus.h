#pragma once

#include "../../Include/ROS2/Lidar/RaycastResults.h"

#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <ROS2/Lidar/RaycastResults.h>

namespace ROS2
{
    class LidarResultsTransformRequests
    {
    public:
        virtual ~LidarResultsTransformRequests() = default;
        AZ_RTTI(LidarResultsTransformRequests, "{362db8a9-7846-41fd-a692-bb47e531b36a}");

        virtual void TransformResults(RaycastResults& raycastResults) = 0;
    };

    class LidarResultsTransformBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        //////////////////////////////////////////////////////////////////////////
    };

    using LidarResultsTransformBus = AZ::EBus<LidarResultsTransformRequests, LidarResultsTransformBusTraits>;
} // namespace ROS2
