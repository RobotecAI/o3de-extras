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
    ->Field("GaussianNoiseStdDevPct", &NoiseConfig::gaussianNoiseStdDevPct)
    ->Field("RandomNoiseRangePct", &NoiseConfig::randomNoiseRangePct)
    ->Field("StdDevNoisePct", &NoiseConfig::stddevNoisePct);

    serialize->Enum<NoiseType>()
    ->Version(1)
    ->Value("Gaussian", NoiseType::Gaussian)
    ->Value("StdDev", NoiseType::StdDev)
    ->Value("Random", NoiseType::Random);

    if (AZ::EditContext * ec = serialize->GetEditContext()) {
      ec->Class<NoiseConfig>("GNSS Noise Configuration", "Configuration for GNSS noise")
      ->DataElement(
        AZ::Edit::UIHandlers::Default, &NoiseConfig::gaussianNoiseStdDevPct,
        "Gaussian Noise Std Dev Percentage",
        "Standard deviation as a percentage of the value for Gaussian noise")
      ->DataElement(
        AZ::Edit::UIHandlers::Default, &NoiseConfig::randomNoiseRangePct,
        "Random Noise Range Percentage",
        "Range as a percentage of the value for random noise")
      ->DataElement(
        AZ::Edit::UIHandlers::Default, &NoiseConfig::stddevNoisePct,
        "Std Dev Noise Percentage",
        "Standard deviation as a percentage of the value for noise");

      ec->Enum<NoiseType>("Noise Type", "The type of noise to apply")
      ->Value("Gaussian", NoiseType::Gaussian)
      ->Value("StdDev", NoiseType::StdDev)
      ->Value("Random", NoiseType::Random);
    }
  }
}

GNSSPostProcessing::GNSSPostProcessing()
: gen(std::mt19937(rd()))
{
  gaussianDist = std::normal_distribution<>(0.0, 0.01);       // 1% standard deviation
  stddevDist = std::normal_distribution<>(0.0, 0.01);       // 1% standard deviation
  randomDist = std::uniform_real_distribution<>(-0.01, 0.01);       // +/-1% range
}

GNSSPostProcessing::GNSSPostProcessing(NoiseConfig noiseConfig)
: gen(std::mt19937(rd())), noiseConfig(noiseConfig)
{
}

double GNSSPostProcessing::GaussianNoise(double value)
{
  // Apply Gaussian noise as a percentage of the current value
  double noise = gaussianDist(gen) * value * noiseConfig.gaussianNoiseStdDevPct;
  return value + noise;
}

double GNSSPostProcessing::RandomNoise(double value)
{
  // Apply random noise within a range based on a percentage of the current value
  double range = value * noiseConfig.randomNoiseRangePct;
  std::uniform_real_distribution<> localRandomDist(-range, range);
  double noise = localRandomDist(gen);
  return value + noise;
}

double GNSSPostProcessing::StdDevNoise(double value)
{
  // Apply standard deviation noise as a percentage of the current value
  double noise = stddevDist(gen) * value * noiseConfig.stddevNoisePct;
  return value + noise;
}

} // namespace ROS2
