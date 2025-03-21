/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

namespace SimulationInterfaces
{
    //! # A set of filters to apply to entity queries. See GetEntities, GetEntitiesStates.
    //! # The filters are combined with a logical AND.
    struct EntityFilter
    {
        AZStd::string m_filter; //! A posix regular expression to match against entity names
        AZStd::shared_ptr<Physics::ShapeConfiguration> m_bounds; //! A shape to use for filtering entities, null means no bounds filtering

    };
    class SimulationInterfacesRequests
    {
    public:
        AZ_RTTI(SimulationInterfacesRequests, SimulationInterfacesRequestsTypeId);
        virtual ~SimulationInterfacesRequests() = default;

        //! # Get a list of entities that match the filter.
        //! context : https://github.com/adamdbrw/simulation_interfaces/blob/simulation_interfaces/srv/GetEntities.srv
        virtual AZStd::vector<AZStd::string> GetEntities(const EntityFilter& filter) = 0;

    };

    class SimulationInterfacesBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////

    };

    using SimulationInterfacesRequestBus = AZ::EBus<SimulationInterfacesRequests, SimulationInterfacesBusTraits>;
    using SimulationInterfacesInterface = AZ::Interface<SimulationInterfacesRequests>;

} // namespace SimulationInterfaces
