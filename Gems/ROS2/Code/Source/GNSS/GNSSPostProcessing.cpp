/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GNSSPostProcessing.h"
#include <random>


namespace ROS2
{
    GNSSPostProcessing::GNSSPostProcessing()
    {
        gen = std::mt19937(rd());
        gaussianDist = std::normal_distribution<>(0.0, 1.0);
        stddevNoise = 1.0;
        randomDist = std::uniform_real_distribution<>(-1.0, 1.0);
    }

    double GNSSPostProcessing::GaussianNoise(double value)
    {
        return value + gaussianDist(gen);
    }

    double GNSSPostProcessing::StdDevNoise(double value)
    {
        return value + stddevNoise;
    }

    double GNSSPostProcessing::RandomNoise(double value)
    {
        return value + randomDist(gen);
    }

} // namespace ROS2