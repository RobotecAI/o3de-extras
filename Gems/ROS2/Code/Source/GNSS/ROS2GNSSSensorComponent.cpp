/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2GNSSSensorComponent.h"
#include <AzCore/Math/Matrix4x4.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Names.h>

#include "Georeference/GNSSFormatConversions.h"
#include <ROS2/Georeference/GeoreferenceBus.h>


namespace ROS2
{
namespace
{
const char * GNSSMsgType = "sensor_msgs::msg::NavSatFix";
}

void ROS2GNSSSensorComponent::Reflect(AZ::ReflectContext * context)
{
  if (auto * serialize = azrtti_cast<AZ::SerializeContext *>(context)) {
    serialize->Class<ROS2GNSSSensorComponent, SensorBaseType>()
    ->Version(4)
    ->Field("ApplyPostProcessing", &ROS2GNSSSensorComponent::m_applyPostProcessing)
    ->Field("NoiseType", &ROS2GNSSSensorComponent::m_noiseType)
    ->Field("NoiseConfiguration", &ROS2GNSSSensorComponent::m_noiseConfig);

    if (auto * editContext = serialize->GetEditContext()) {
      editContext->Class<ROS2GNSSSensorComponent>("ROS2 GNSS Sensor", "GNSS sensor component")
      ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
      ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
      ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
      ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2GNSSSensor.svg")
      ->Attribute(
        AZ::Edit::Attributes::ViewportIcon,
        "Editor/Icons/Components/Viewport/ROS2GNSSSensor.svg")
      ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
      ->DataElement(
        AZ::Edit::UIHandlers::Default, &ROS2GNSSSensorComponent::m_applyPostProcessing,
        "Apply Post Processing", "Apply post processing to the GNSS message")
      ->Attribute(
        AZ::Edit::Attributes::ChangeNotify,
        &ROS2GNSSSensorComponent::OnPostProcessingChanged)
      ->DataElement(
        AZ::Edit::UIHandlers::Default, &ROS2GNSSSensorComponent::m_noiseConfig,
        "Noise Configuration", "Configuration for GNSS noise")
      ->DataElement(
        AZ::Edit::UIHandlers::ComboBox, &ROS2GNSSSensorComponent::m_noiseType,
        "Noise Type", "The type of noise to apply")
      ->EnumAttribute(NoiseType::Gaussian, "Gaussian")
      ->EnumAttribute(NoiseType::StdDev, "Standard Deviation")
      ->EnumAttribute(NoiseType::Random, "Random");
    }
  }
}

ROS2GNSSSensorComponent::ROS2GNSSSensorComponent()
{
  TopicConfiguration pc;
  pc.m_type = GNSSMsgType;
  pc.m_topic = "gnss";
  m_sensorConfiguration.m_frequency = 10;
  m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(GNSSMsgType, pc));
}

ROS2GNSSSensorComponent::ROS2GNSSSensorComponent(const SensorConfiguration & sensorConfiguration)
{
  m_sensorConfiguration = sensorConfiguration;
}

void ROS2GNSSSensorComponent::Activate()
{
  m_gnssPostProcessing = AZStd::make_unique<GNSSPostProcessing>(m_noiseConfig);
  GNSSPostProcessingRequestBus::Handler::BusConnect(GetEntityId());
  auto ros2Node = ROS2Interface::Get()->GetNode();
  AZ_Assert(
    m_sensorConfiguration.m_publishersConfigurations.size() == 1,
    "Invalid configuration of publishers for GNSS sensor");

  const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[GNSSMsgType];
  const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
  m_gnssPublisher = ros2Node->create_publisher<sensor_msgs::msg::NavSatFix>(
    fullTopic.data(), publisherConfig.GetQoS());

  m_gnssMsg.header.frame_id = "gnss_frame_id";

  StartSensor(
    m_sensorConfiguration.m_frequency,
    [this]([[maybe_unused]] auto &&... args)
    {
      if (!m_sensorConfiguration.m_publishingEnabled) {
        return;
      }
      FrequencyTick();
    });
}

void ROS2GNSSSensorComponent::Deactivate()
{
  StopSensor();
  m_gnssPublisher.reset();
  GNSSPostProcessingRequestBus::Handler::BusDisconnect();
}

bool ROS2GNSSSensorComponent::GetFixState()
{
  return m_isFix;
}

void ROS2GNSSSensorComponent::SetFixState(bool isFix)
{
  m_isFix = isFix;
}

void ROS2GNSSSensorComponent::ToggleFixLoss()
{
  m_isFix = !m_isFix;
}

void ROS2GNSSSensorComponent::FrequencyTick()
{
  UpdateGnssMessage();       // Update the message with the current data
  ApplyPostProcessing(m_gnssMsg);       // Apply post-processing if required
  m_gnssPublisher->publish(m_gnssMsg);       // Publish the possibly modified message
}

void ROS2GNSSSensorComponent::UpdateGnssMessage()
{
  if (!m_isFix) {
    // Simulate fix loss
    m_gnssMsg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    m_gnssMsg.latitude = std::numeric_limits<double>::quiet_NaN();
    m_gnssMsg.longitude = std::numeric_limits<double>::quiet_NaN();
    m_gnssMsg.altitude = std::numeric_limits<double>::quiet_NaN();
  } else {
    // Normal operation, update current position
    AZ::Vector3 currentPosition{0.0f};
    AZ::TransformBus::EventResult(
      currentPosition,
      GetEntityId(), &AZ::TransformBus::Events::GetWorldTranslation);

    WGS::WGS84Coordinate currentPositionWGS84;
    ROS2::GeoreferenceRequestsBus::BroadcastResult(
      currentPositionWGS84, &GeoreferenceRequests::ConvertFromLevelToWSG84, currentPosition);

    m_gnssMsg.latitude = currentPositionWGS84.m_latitude;
    m_gnssMsg.longitude = currentPositionWGS84.m_longitude;
    m_gnssMsg.altitude = currentPositionWGS84.m_altitude;
    m_gnssMsg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
    m_gnssMsg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  }
}

void ROS2GNSSSensorComponent::ApplyPostProcessing(sensor_msgs::msg::NavSatFix & gnss)
{
  if (!m_applyPostProcessing) {
    return;
  }

  switch (static_cast<NoiseType>(m_noiseType)) {
    case NoiseType::Gaussian:
      gnss.latitude = m_gnssPostProcessing->GaussianNoise(gnss.latitude);
      gnss.longitude = m_gnssPostProcessing->GaussianNoise(gnss.longitude);
      gnss.altitude = m_gnssPostProcessing->GaussianNoise(gnss.altitude);
      break;
    case NoiseType::StdDev:
      gnss.latitude = m_gnssPostProcessing->StdDevNoise(gnss.latitude);
      gnss.longitude = m_gnssPostProcessing->StdDevNoise(gnss.longitude);
      gnss.altitude = m_gnssPostProcessing->StdDevNoise(gnss.altitude);
      break;
    case NoiseType::Random:
      gnss.latitude = m_gnssPostProcessing->RandomNoise(gnss.latitude);
      gnss.longitude = m_gnssPostProcessing->RandomNoise(gnss.longitude);
      gnss.altitude = m_gnssPostProcessing->RandomNoise(gnss.altitude);
      break;
    default:
      break;
  }
}

bool ROS2GNSSSensorComponent::IsPostProcessingApplied() const
{
  return m_applyPostProcessing;
}

AZ::Crc32 ROS2GNSSSensorComponent::OnPostProcessingChanged() const
{
  // Logic to enable/disable other fields based on the value of m_applyPostProcessing
  return m_applyPostProcessing ? AZ::Edit::PropertyRefreshLevels::EntireTree : AZ::Edit::
         PropertyRefreshLevels::None;
}

} // namespace ROS2
