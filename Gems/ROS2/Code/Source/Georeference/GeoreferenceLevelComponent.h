/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Georeference/GeoreferenceBus.h>

namespace ROS2
{

    class GeoReferenceLevelComponent
        : public AZ::Component
        , private GeoreferenceRequestsBus::Handler
        , private AZ::EntityBus::Handler

    {
    public:
        AZ_COMPONENT(GeoReferenceLevelComponent, "{7dcd0112-db23-41b8-90b8-4c66c6a197e4}");
        GeoReferenceLevelComponent() = default;
        ~GeoReferenceLevelComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        // Component overrides
        void Activate() override;
        void Deactivate() override;

    private:
        // EntityBus overrides ...
        void OnEntityActivated(const AZ::EntityId& entityId) override;

        // GeoreferenceRequestsBus::Handler overrides ...
        GeoreferenceRequests::WGS84Coordinate ConvertFromLevelToWSG84(const AZ::Vector3& xyz) override;
        AZ::Vector3 ConvertFromWSG84ToLevel(const GeoreferenceRequests::WGS84Coordinate& latLon) override;
        AZ::Quaternion ConvertFromLevelRotationToENU() override;

        AZ::EntityId m_EnuOriginLocationEntityId; //!< EntityId of the entity that lays in the origin of the ENU coordinate system
        double m_EnuOriginLatitude = 0.0; //!< Latitude of the origin of the ENU coordinate system
        double m_EnuOriginLongitude = 0.0; //!< Longitude of the origin of the ENU coordinate system
        double m_EnuOriginAltitude = 0.0; //!< Altitude of the origin of the ENU coordinate system

        AZ::Transform m_EnuOriginTransform; //!< Transform of the entity that lays in the origin of the ENU coordinate system
    };

} // namespace ROS2
