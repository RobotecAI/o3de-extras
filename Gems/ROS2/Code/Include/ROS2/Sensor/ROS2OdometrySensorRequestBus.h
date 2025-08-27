#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/Math/Transform.h>

#include <AzCore/Component/Component.h>
#include <AzCore/EBus/EBus.h>

namespace ROS2
{
    struct Odometry
    {
        AZ::Vector3 m_twistAngular;
        AZ::Vector3 m_twistLinear;
        AZ::Transform m_pose;
    };

    class ROS2OdometrySensorRequests : public AZ::ComponentBus
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;

        AZ_RTTI(ROS2OdometrySensorRequests, "{90044f78-ffb9-4d1a-851d-1755a7146047}");

        virtual Odometry GetOdometry() = 0;
    };

    using ROS2OdometrySensorRequestBus = AZ::EBus<ROS2OdometrySensorRequests>;
} // nnamespace ROS2