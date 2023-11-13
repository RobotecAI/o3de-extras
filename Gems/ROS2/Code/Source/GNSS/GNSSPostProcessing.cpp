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
    ->Field("GaussianMin", &NoiseConfig::gaussianMin)
    ->Field("GaussianMax", &NoiseConfig::gaussianMax)
    ->Field("RandomMin", &NoiseConfig::randomMin)
    ->Field("RandomMax", &NoiseConfig::randomMax)
    ->Field("StdDev", &NoiseConfig::stddev);

    serialize->Enum<NoiseType>()
    ->Version(1)
    ->Value("Gaussian", NoiseType::Gaussian)
    ->Value("StdDev", NoiseType::StdDev)
    ->Value("Random", NoiseType::Random);

    if (AZ::EditContext * ec = serialize->GetEditContext()) {
      ec->Class<NoiseConfig>("GNSS Noise Configuration", "Configuration for GNSS noise")
      ->DataElement(
        AZ::Edit::UIHandlers::Default, &NoiseConfig::gaussianMin, "Gaussian Min",
        "Minimum value for Gaussian noise")
      ->DataElement(
        AZ::Edit::UIHandlers::Default, &NoiseConfig::gaussianMax, "Gaussian Max",
        "Maximum value for Gaussian noise")
      ->DataElement(
        AZ::Edit::UIHandlers::Default, &NoiseConfig::randomMin, "Random Min",
        "Minimum value for random noise")
      ->DataElement(
        AZ::Edit::UIHandlers::Default, &NoiseConfig::randomMax, "Random Max",
        "Maximum value for random noise")
      ->DataElement(
        AZ::Edit::UIHandlers::Default, &NoiseConfig::stddev, "Standard Deviation",
        "Standard deviation for noise");
    }
  }
}

GNSSPostProcessing::GNSSPostProcessing()
: gen(std::mt19937(rd()))
{
  // Initialize distributions with default range values
  gaussianDist = std::normal_distribution<>(noiseConfig.gaussianMin, noiseConfig.gaussianMax);
  stddevDist = std::normal_distribution<>(0.0, noiseConfig.stddev);
  randomDist = std::uniform_real_distribution<>(noiseConfig.randomMin, noiseConfig.randomMax);
}

GNSSPostProcessing::GNSSPostProcessing(NoiseConfig noiseConfig)
: gen(std::mt19937(rd())), noiseConfig(noiseConfig)
{
  // Initialize distributions with noise configuration values
  gaussianDist = std::normal_distribution<>(noiseConfig.gaussianMin, noiseConfig.gaussianMax);
  stddevDist = std::normal_distribution<>(0.0, noiseConfig.stddev);
  randomDist = std::uniform_real_distribution<>(noiseConfig.randomMin, noiseConfig.randomMax);
}

double GNSSPostProcessing::GaussianNoise(double value)
{
  // Apply Gaussian noise using the configured range
  double noise = gaussianDist(gen);
  return value + noise;
}

double GNSSPostProcessing::RandomNoise(double value)
{
  // Apply random noise using the configured range
  double noise = randomDist(gen);
  return value + noise;
}

double GNSSPostProcessing::StdDevNoise(double value)
{
  // Apply noise based on standard deviation
  double noise = stddevDist(gen);
  return value + noise;
}

} // namespace ROS2
