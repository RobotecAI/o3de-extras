/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GNSSPostProcessing.h"
#include <random>
#include <AzCore/Serialization/EditContext.h>

namespace ROS2
{
void NoiseConfig::Reflect(AZ::ReflectContext * context)
{
  if (AZ::SerializeContext * serialize = azrtti_cast<AZ::SerializeContext *>(context)) {
    serialize->Class<NoiseConfig>()
    ->Version(1)
    ->Field("StdDevX", &NoiseConfig::stddevX)
    ->Field("StdDevY", &NoiseConfig::stddevY)
    ->Field("StdDevZ", &NoiseConfig::stddevZ);

    if (AZ::EditContext * ec = serialize->GetEditContext()) {
      ec->Class<NoiseConfig>("GNSS Noise Configuration", "Configuration for GNSS noise")
      ->DataElement(
        AZ::Edit::UIHandlers::Default, &NoiseConfig::stddevX, "Std Dev X",
        "Standard deviation for noise in X direction")
      ->DataElement(
        AZ::Edit::UIHandlers::Default, &NoiseConfig::stddevY, "Std Dev Y",
        "Standard deviation for noise in Y direction")
      ->DataElement(
        AZ::Edit::UIHandlers::Default, &NoiseConfig::stddevZ, "Std Dev Z",
        "Standard deviation for noise in Z direction");
    }
  }
}

GNSSPostProcessing::GNSSPostProcessing()
: gen(std::mt19937(rd())),
  gaussianDistX(0.0, noiseConfig.stddevX),
  gaussianDistY(0.0, noiseConfig.stddevY),
  gaussianDistZ(0.0, noiseConfig.stddevZ)
{
}

GNSSPostProcessing::GNSSPostProcessing(NoiseConfig noiseConfig)
: gen(std::mt19937(rd())), noiseConfig(noiseConfig),
  gaussianDistX(0.0, noiseConfig.stddevX),
  gaussianDistY(0.0, noiseConfig.stddevY),
  gaussianDistZ(0.0, noiseConfig.stddevZ)
{
}

double GNSSPostProcessing::GaussianNoise(double value, double stddev)
{
  std::normal_distribution<> dist(0.0, stddev);
  return value + dist(gen);
}

void GNSSPostProcessing::PostProcessGNSS(sensor_msgs::msg::NavSatFix & gnss)
{
  gnss.latitude += GaussianNoise(0, noiseConfig.stddevX);
  gnss.longitude += GaussianNoise(0, noiseConfig.stddevY);
  gnss.altitude += GaussianNoise(0, noiseConfig.stddevZ);
}

} // namespace ROS2
