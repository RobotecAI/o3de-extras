/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AzCore/EBus/Policies.h"
#include "AzCore/std/string/string.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    class RobotImporterRequest : public AZ::EBusTraits
    {
    public:

        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;

        //! Generate prefab from urdf file, it is asynchronous call.
        //! @param filePath The path of the urdf file
        //! @param importAssetWithUrdf If true, the assets referenced in the urdf file will be imported
        //! @param useArticulation If true, the prefab will be generated with articulation
        virtual void GeneratePrefabFromFile(const AZStd::string_view filePath, bool importAssetWithUrdf, bool useArticulation) = 0;

        //! Check if the prefab generation is done.
        virtual bool IsDone() = 0;

        //! Check if the prefab generation is successful.
        virtual bool IsSuccessful() = 0;
    };

    using RobotImporterRequestBus = AZ::EBus<RobotImporterRequest>;

    class RobotImporterRequestHandler
        : public RobotImporterRequestBus::Handler
        , public AZ::BehaviorEBusHandler
    {
    public:
        AZ_EBUS_BEHAVIOR_BINDER(
            RobotImporterRequestHandler,
            "{f99cf3b1-650f-4f72-b3e8-9cf137e0d264}",
            AZ::SystemAllocator,
            GeneratePrefabFromFile,
            IsDone,
            IsSuccessful);

        static void Reflect(AZ::ReflectContext* context)
        {
            if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
            {
                behaviorContext->EBus<RobotImporterRequestBus>("RobotImporterBus")
                    ->Attribute(AZ::Script::Attributes::Category, "Robotics")
                    ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Automation)
                    ->Attribute(AZ::Script::Attributes::Module, "ros2")
                    ->Handler<RobotImporterRequestHandler>()
                    ->Event("ImportURDF", &RobotImporterRequestBus::Events::GeneratePrefabFromFile)
                    ->Event("IsDone", &RobotImporterRequestBus::Events::IsDone)
                    ->Event("IsSuccessful",&RobotImporterRequestBus::Events::IsSuccessful)
                    ;
            }
        }

        void GeneratePrefabFromFile(const AZStd::string_view filePath, bool importAssetWithUrdf, bool useArticulation) override
        {
            Call(FN_GeneratePrefabFromFile, filePath, importAssetWithUrdf, useArticulation);
        }

        bool IsDone() override
        {
            bool result = false;
            CallResult(result, FN_IsDone);
            return result;
        }

        bool IsSuccessful() override
        {
            bool result = false;
            CallResult(result, FN_IsSuccessful);
            return result;
        }
    };
} // namespace ROS2