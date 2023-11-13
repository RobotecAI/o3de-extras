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

enum class NoiseType // Placeholder for future noise types
{
  Gaussian
};

struct NoiseConfig
{
  AZ_TYPE_INFO(NoiseConfig, "{A4D41EFF-49A1-4B72-A382-70734FD6CE03}");
  static void Reflect(AZ::ReflectContext * context);
  double stddevX = 0.0001;
  double stddevY = 0.0001;
  double stddevZ = 0.0001;
};

class GNSSPostProcessing
{
public:
  AZ_TYPE_INFO(GNSSPostProcessing, "{7291A82E-A394-4CB6-965E-E3C65667A437}");

  GNSSPostProcessing();
  GNSSPostProcessing(NoiseConfig noiseConfig);

  virtual ~GNSSPostProcessing() = default;

  void PostProcessGNSS(sensor_msgs::msg::NavSatFix & gnss);

  double GaussianNoise(double value, double stddev);

private:
  std::random_device rd;
  std::mt19937 gen;
  NoiseConfig noiseConfig{};
  std::normal_distribution<> gaussianDistX;
  std::normal_distribution<> gaussianDistY;
  std::normal_distribution<> gaussianDistZ;
};

} // namespace ROS2