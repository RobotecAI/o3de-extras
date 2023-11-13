/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <random>
#include "Georeference/GNSSFormatConversions.h"
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace ROS2
{

enum class NoiseType
{
  Gaussian,
  StdDev,
  Random
};

struct NoiseConfig
{
  AZ_TYPE_INFO(NoiseConfig, "{A4D41EFF-49A1-4B72-A382-70734FD6CE03}");
  static void Reflect(AZ::ReflectContext * context);

  double gaussianNoiseStdDevPct = 0.01; // Standard deviation as a percentage of the value
  double randomNoiseRangePct = 0.01;      // Random noise range as a percentage of the value
  double stddevNoisePct = 0.01;           // Standard deviation noise as a percentage of the value
};

class GNSSPostProcessing
{
public:
  AZ_TYPE_INFO(GNSSPostProcessing, "{7291A82E-A394-4CB6-965E-E3C65667A437}");

  GNSSPostProcessing();
  GNSSPostProcessing(NoiseConfig noiseConfig);


  virtual ~GNSSPostProcessing() = default;

  void PostProcessGNSS(sensor_msgs::msg::NavSatFix & gnss) const;

  double GaussianNoise(double value);
  double StdDevNoise(double value);
  double RandomNoise(double value);

private:
  std::random_device rd;
  std::mt19937 gen;
  NoiseConfig noiseConfig{};
  std::normal_distribution<> gaussianDist;
  std::uniform_real_distribution<> randomDist;
  std::normal_distribution<> stddevDist;
};

} // namespace ROS2
