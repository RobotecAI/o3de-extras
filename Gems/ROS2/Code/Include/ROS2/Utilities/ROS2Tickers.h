/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzFramework/Physics/Common/PhysicsEvents.h>

namespace ROS2::Tickers
{
    //! Connects to OnSceneActiveSimulatedBodiesEvent to give an opportunity
    //! of computation after step of physics simulation.
    //! Derive this class for very frequent refreshing state e.g. sensors
    class SimulatedBodyEventTicker
    {
    public:
        SimulatedBodyEventTicker() = default;
        ~SimulatedBodyEventTicker() = default;
    protected:
        void ConnectToEvent();
        void DisconnectFromEvent();
    private:
        //! Executes the sensor action (acquire data -> publish) according to frequency.
        //! Override to implement a specific sensor behavior.
        //! @param deltaTime time elapsed from the last call in seconds.
        virtual void OnEventTick(float deltaTime) {};

        AzPhysics::SceneEvents::OnSceneActiveSimulatedBodiesEvent::Handler m_simulatedBodiesEventHandler;
    };
} // namespace ROS2::Tickers
