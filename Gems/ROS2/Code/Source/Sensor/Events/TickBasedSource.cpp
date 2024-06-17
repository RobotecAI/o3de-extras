/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Sensor/SensorConfiguration.h>

namespace ROS2
{
    void TickBasedSource::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<TickBasedSource>()->Version(1);
        }
    }

    void TickBasedSource::Start()
    {
        m_lastMonotonicTime = AZStd::chrono::steady_clock::now();
        AZ::SystemTickBus::Handler::BusConnect();
    }

    void TickBasedSource::Stop()
    {
        AZ::SystemTickBus::Handler::BusDisconnect();
    }

    float TickBasedSource::GetDeltaTime() const
    {
        return m_deltaTime;
    }

    void TickBasedSource::OnSystemTick()
    {
        auto currentMonotonicTime = AZStd::chrono::steady_clock::now();
        m_deltaTime = AZStd::chrono::duration<float>(currentMonotonicTime - m_lastMonotonicTime).count();
        m_lastMonotonicTime = currentMonotonicTime;
        m_sourceEvent.Signal();

    }
} // namespace ROS2
