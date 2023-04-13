/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ROS2/Utilities/ROS2Tickers.h>
#include <AzCore/Interface/Interface.h>
#include <AzFramework/Physics/PhysicsScene.h>

namespace ROS2::Tickers
{
    void SimulatedBodyEventTicker::ConnectToEvent()
    {
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);

        m_simulatedBodiesEventHandler = AzPhysics::SceneEvents::OnSceneActiveSimulatedBodiesEvent::Handler(
            [this](AzPhysics::SceneHandle sceneHandle,
                    const AzPhysics::SimulatedBodyHandleList& activeBodyList,
                    float deltaTime)
            {
                    OnEventTick(deltaTime);
            });

        sceneInterface->RegisterSceneActiveSimulatedBodiesHandler(sceneHandle, m_simulatedBodiesEventHandler);
    }

    void SimulatedBodyEventTicker::DisconnectFromEvent()
    {
        m_simulatedBodiesEventHandler.Disconnect();
    }

} // namespace ROS2::Tickers
