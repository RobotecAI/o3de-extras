/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GeoreferenceLevelComponent.h"
#include <ROS2/Georeference/GeoreferenceStructures.h>
#include "GNSSFormatConversions.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Matrix4x4.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
namespace ROS2
{

    void GeoReferenceLevelComponent::Reflect(AZ::ReflectContext* context)
    {
        WGS::WGS84Coordinate::Reflect(context);
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GeoReferenceLevelComponent>()
                ->Version(1)
                ->Field("EnuOriginWSG84", &GeoReferenceLevelComponent::m_OriginLocation)
                ->Field("EnuOriginLocationEntityId", &GeoReferenceLevelComponent::m_EnuOriginLocationEntityId);

            if (auto* editContext = serialize->GetEditContext())
            {
                editContext
                    ->Class<GeoReferenceLevelComponent>(
                        "GeoReference Level Component", "Component allows to provide georeference level for the level")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Level"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2GNSSSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2GNSSSensor.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoReferenceLevelComponent::m_EnuOriginLocationEntityId,
                        "ENU Origin Transform",
                        "ENU (East-North-Up) origin in the level")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoReferenceLevelComponent::m_OriginLocation,
                        "ENU Origin Coordinates in WGS84","ENU Origin Coordinates in WGS84");
            }
        }
    }

    void GeoReferenceLevelComponent::Activate()
    {
        AZ::EntityBus::Handler::BusConnect(m_EnuOriginLocationEntityId);
        GeoreferenceRequestsBus::Handler::BusConnect();
    }

    void GeoReferenceLevelComponent::Deactivate()
    {
        GeoreferenceRequestsBus::Handler::BusDisconnect();
        AZ::EntityBus::Handler::BusDisconnect();
    }

    void GeoReferenceLevelComponent::OnEntityActivated(const AZ::EntityId& entityId)
    {
        m_EnuOriginTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(m_EnuOriginTransform, m_EnuOriginLocationEntityId, &AZ::TransformBus::Events::GetWorldTM);
        m_EnuOriginTransform.Invert();
        AZ::EntityBus::Handler::BusDisconnect();
    }

    WGS::WGS84Coordinate GeoReferenceLevelComponent::ConvertFromLevelToWSG84(const AZ::Vector3& xyz)
    {
        using namespace ROS2::GNSS;
        const auto enu = WGS::Vector3d(m_EnuOriginTransform.TransformPoint(xyz));
        const auto ecef =
            ENUToECEF(m_OriginLocation,enu);
        return ECEFToWGS84(ecef);
    }

    AZ::Vector3 GeoReferenceLevelComponent::ConvertFromWSG84ToLevel(const WGS::WGS84Coordinate& latLon)
    {
        using namespace ROS2::GNSS;
        const auto ecef = WGS84ToECEF(latLon);
        const auto enu = ECEFToENU(m_OriginLocation, ecef);
        return m_EnuOriginTransform.TransformPoint(enu.ToVector3f());
    };

    AZ::Quaternion GeoReferenceLevelComponent::ConvertFromLevelRotationToENU()
    {
        return m_EnuOriginTransform.GetRotation();
    };

} // namespace ROS2
