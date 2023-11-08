/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Georeference/GNSSFormatConversions.h"

constexpr double earthSemimajorAxis = 6378137.0;
constexpr double reciprocalFlattening = 1.0 / 298.257223563;
constexpr double earthSemiminorAxis = earthSemimajorAxis * (1.0 - reciprocalFlattening);
constexpr double firstEccentricitySquared = 2.0 * reciprocalFlattening - reciprocalFlattening * reciprocalFlattening;
constexpr double secondEccentrictySquared =
    reciprocalFlattening * (2.0 - reciprocalFlattening) / ((1.0 - reciprocalFlattening) * (1.0 - reciprocalFlattening));

// Based on http://wiki.gis.com/wiki/index.php/Geodetic_system
namespace ROS2::GNSS
{
    AZStd::array<double,3> FromAzVector3(const AZ::Vector3& vector3)
    {
        return { vector3.GetX(), vector3.GetY(), vector3.GetZ() };
    }
    AZ::Vector3 ToAzVector3(const AZStd::array<double,3>& array)
    {
        return {static_cast<float>(array[0]), static_cast<float>(array[1]), static_cast<float>(array[2])};
    }

    double DegToRad(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    double RadToDeg(double radians)
    {
        return radians * 180.0 / M_PI;
    }

    AZStd::array<double,3> WGS84ToECEF(const AZStd::array<double,3>& latitudeLongitudeAltitude)
    {
        const double latitudeRad = DegToRad(latitudeLongitudeAltitude[0]);
        const double longitudeRad = DegToRad(latitudeLongitudeAltitude[1]);
        const double altitude = latitudeLongitudeAltitude[2];

        const double helper = std::sqrt(1.0f - firstEccentricitySquared * std::sin(latitudeRad) * std::sin(latitudeRad));

        const double X = (earthSemimajorAxis / helper + altitude) * std::cos(latitudeRad) * std::cos(longitudeRad);
        const double Y = (earthSemimajorAxis / helper + altitude) * std::cos(latitudeRad) * std::sin(longitudeRad);
        const double Z = (earthSemimajorAxis * (1.0 - firstEccentricitySquared) / helper + altitude) * std::sin(latitudeRad);

        return { X, Y, Z };
    }

    AZStd::array<double,3> ECEFToENU(const AZStd::array<double,3>& referenceLatitudeLongitudeAltitude, const AZStd::array<double,3>& ECEFPoint)
    {
        const AZStd::array<double,3> referencePointInECEF = WGS84ToECEF(referenceLatitudeLongitudeAltitude);
        const AZStd::array<double,3> pointToReferencePointECEF = {ECEFPoint[0] - referencePointInECEF[0],
                                                                    ECEFPoint[1] - referencePointInECEF[1],
                                                                    ECEFPoint[2] - referencePointInECEF[2]};

        const double referenceLatitudeRad = DegToRad(referenceLatitudeLongitudeAltitude[0]);
        const double referenceLongitudeRad = DegToRad(referenceLatitudeLongitudeAltitude[1]);

        return {
            -sin(referenceLongitudeRad) * pointToReferencePointECEF[0] +
                cos(referenceLongitudeRad) * pointToReferencePointECEF[1],
            -sin(referenceLatitudeRad) * cos(referenceLongitudeRad) * pointToReferencePointECEF[0] -
                sin(referenceLatitudeRad) * sin(referenceLongitudeRad) * pointToReferencePointECEF[1] +
                cos(referenceLatitudeRad) * pointToReferencePointECEF[2],
            cos(referenceLatitudeRad) * cos(referenceLongitudeRad) * pointToReferencePointECEF[0] +
                cos(referenceLatitudeRad) * sin(referenceLongitudeRad) * pointToReferencePointECEF[1] +
                sin(referenceLatitudeRad) * pointToReferencePointECEF[2],
        };
    }

    AZStd::array<double,3> ENUToECEF(const AZStd::array<double,3>& referenceLatitudeLongitudeAltitude, const AZStd::array<double,3>& ENUPoint)
    {
        AZStd::array<double,3> referenceECEF = WGS84ToECEF(referenceLatitudeLongitudeAltitude);

        const double referenceLatitudeRad = DegToRad(referenceLatitudeLongitudeAltitude[0]);
        const double referenceLongitudeRad = DegToRad(referenceLatitudeLongitudeAltitude[1]);
        const double& e = ENUPoint[0];
        const double& n = ENUPoint[1];
        const double& u = ENUPoint[2];

        return { -std::sin(referenceLongitudeRad) * e - std::cos(referenceLongitudeRad) * std::sin(referenceLatitudeRad) * n +
                     std::cos(referenceLongitudeRad) * std::cos(referenceLatitudeRad) * u + referenceECEF[0],
                 std::cos(referenceLongitudeRad) * e - std::sin(referenceLongitudeRad) * std::sin(referenceLatitudeRad) * n +
                     std::cos(referenceLatitudeRad) * std::sin(referenceLongitudeRad) * u + referenceECEF[1],
                 std::cos(referenceLatitudeRad) * n + std::sin(referenceLatitudeRad) * u + referenceECEF[2] };
    }

    AZStd::array<double,3> ECEFToWGS84(const AZStd::array<double,3>& ECFEPoint)
    {
        const double& x = ECFEPoint[0];
        const double& y = ECFEPoint[1];
        const double& z = ECFEPoint[2];

        const double radiusSquared = x * x + y * y;
        const double radius = std::sqrt(radiusSquared);

        const double E2 = earthSemimajorAxis * earthSemimajorAxis - earthSemiminorAxis * earthSemiminorAxis;
        const double F = 54.0 * earthSemiminorAxis * earthSemiminorAxis * z * z;
        const double G = radiusSquared + (1.0 - firstEccentricitySquared) * z * z - firstEccentricitySquared * E2;
        const double c = (firstEccentricitySquared * firstEccentricitySquared * F * radiusSquared) / (G * G * G);
        const double s = std::pow(1. + c + std::sqrt(c * c + 2. * c), 1. / 3);
        const double P = F / (3.0 * (s + 1.0 / s + 1.0) * (s + 1.0 / s + 1.0) * G * G);
        const double Q = std::sqrt(1.0 + 2.0 * firstEccentricitySquared * firstEccentricitySquared * P);

        const double ro = -(firstEccentricitySquared * P * radius) / (1.0 + Q) +
            std::sqrt(
                (earthSemimajorAxis * earthSemimajorAxis / 2.0) * (1.0 + 1.0 / Q) -
                ((1.0 - firstEccentricitySquared) * P * z * z) / (Q * (1.0 + Q)) - P * radiusSquared / 2.0);
        const double tmp = (radius - firstEccentricitySquared * ro) * (radius - firstEccentricitySquared * ro);
        const double U = std::sqrt(tmp + z * z);
        const double V = std::sqrt(tmp + (1.0 - firstEccentricitySquared) * z * z);
        const double zo = (earthSemiminorAxis * earthSemiminorAxis * z) / (earthSemimajorAxis * V);

        const double latitude = std::atan((z + secondEccentrictySquared * zo) / radius);
        const double longitude = std::atan2(y, x);
        const double altitude = U * (1.0 - earthSemiminorAxis * earthSemiminorAxis / (earthSemimajorAxis * V));

        return { RadToDeg(latitude), RadToDeg(longitude), altitude };
    }

} // namespace ROS2::GNSS
