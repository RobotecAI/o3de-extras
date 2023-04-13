/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "SensorConfiguration.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>

namespace ROS2
{
    //! Captures common behavior of ROS2 sensor Components.
    //! Sensors acquire data from the simulation engine and publish it to ROS2 ecosystem.
    //! Derive this Component to implement a new ROS2 sensor. Each sensor Component requires ROS2FrameComponent.
    class ROS2SensorComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        ROS2SensorComponent() = default;
        virtual ~ROS2SensorComponent() = default;
        AZ_COMPONENT(ROS2SensorComponent, "{91BCC1E9-6D93-4466-9CDB-E73D497C6B5E}");

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

        static void Reflect(AZ::ReflectContext* context);
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

    protected:
        AZStd::string GetNamespace() const; //!< Get a complete namespace for this sensor topics and frame ids.
        AZStd::string GetFrameID() const; //!< Returns this sensor frame ID. The ID contains namespace.

        //! Check if execution deadline has arrived.
        //! This function needs to be called every loop's iteration (eg TickBus::Handler::OnTick).
        //! @param expectedLoopTime the expected time to loop call in seconds.
        //! @returns if measurement should be done/published.
        bool IsPublicationDeadline(float expectedLoopTime);

        SensorConfiguration m_sensorConfiguration;

    private:
        //! Executes the sensor action (acquire data -> publish) according to frequency.
        //! Override to implement a specific sensor behavior.
        //! @param deltaTime time elapsed from the last call in seconds.
        virtual void FrequencyTick() {};

        //! Visualise sensor operation.
        //! For example, draw points or rays for a lidar, viewport for a camera, etc.
        //! Visualisation can be turned on or off in SensorConfiguration.
        virtual void Visualise() {};

        //! The number of ticks that are expected to pass to trigger next measurement.
        AZ::s16 m_tickCountDown = 0;
    };
} // namespace ROS2
