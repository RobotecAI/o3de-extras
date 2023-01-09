/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/VehicleDynamics/DriveModels/PidConfiguration.h>
#include <VehicleDynamics/DriveModel.h>
#include <VehicleDynamics/VehicleConfiguration.h>
#include <VehicleDynamics/VehicleInputsState.h>
#include <VehicleDynamics/WheelDynamicsData.h>
#include <VehicleDynamics/ModelLimits/AckermannModelLimits.h>

namespace ROS2::VehicleDynamics
{
    //! A simple Ackermann system implementation converting speed and steering inputs into wheel impulse and steering element torque
    class AckermannDriveModel : public DriveModel
    {
    public:
        AZ_RTTI(AckermannDriveModel, "{104AC31D-E30B-4454-BF42-4FB37B8CFD9B}", DriveModel);
        DriveModel::DriveModelType DriveType() const override
        {
            return DriveModel::DriveModelType::SimplifiedDriveModelType;
        }
        //////////////////////////////////////////////////////////////////////////
        // DriveModel overrides
        void Activate(const VehicleConfiguration& vehicleConfig) override;
        void SetDisabled(bool isDisabled) override;
        VehicleModelLimits* GetVehicleLimits() override;
        //////////////////////////////////////////////////////////////////////////

        static void Reflect(AZ::ReflectContext* context);

    protected:
        //////////////////////////////////////////////////////////////////////////
        // DriveModel overrides
        void ApplyState(const VehicleInputsState& inputs, uint64_t deltaTimeNs) override;
        //////////////////////////////////////////////////////////////////////////

    private:
        void ApplySteering(float steering, uint64_t deltaTimeNs);
        void ApplySpeed(float speed, uint64_t deltaTimeNs);
        void ApplyWheelSteering(SteeringDynamicsData& wheelData, float steering, double deltaTimeNs);

        VehicleConfiguration m_vehicleConfiguration;
        AZStd::vector<WheelDynamicsData> m_driveWheelsData;
        AZStd::vector<SteeringDynamicsData> m_steeringData;
        PidConfiguration m_steeringPid;
        bool m_disabled{ false };
        float m_steeringDeadZone = 0.01;
        AckermannModelLimits m_limits;
    };
} // namespace ROS2::VehicleDynamics
