/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include "SkidSteeringModelLimits.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2::VehicleDynamics
{
   void SkidSteeringModelLimits::Reflect(AZ::ReflectContext* context)
   {
       if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
       {
           serialize->Class<SkidSteeringModelLimits>()
               ->Version(1)
               ->Field("LinearLimit", &SkidSteeringModelLimits::m_linearLimit)
               ->Field("AngularLimit", &SkidSteeringModelLimits::m_angularLimit);

           if (AZ::EditContext* ec = serialize->GetEditContext())
           {
               ec->Class<SkidSteeringModelLimits>("Skid steering Model Limits", "Limitations of speed, steering angles and other values")
                   ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                   ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                   ->DataElement(AZ::Edit::UIHandlers::Default, &SkidSteeringModelLimits::m_linearLimit, "Linear speed Limit", "Max linear speed (mps)")
                   ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                   ->Attribute(AZ::Edit::Attributes::Max, 100.0f)
                   ->DataElement(
                       AZ::Edit::UIHandlers::Default, &SkidSteeringModelLimits::m_angularLimit, "Angular speed Limit", "Max angular sped (rad/s)")
                   ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                   ->Attribute(AZ::Edit::Attributes::Max, 10.0f);
           }
       }
   }

   float SkidSteeringModelLimits::LimitValue(float value, float absoluteLimit)
   {
       float absoluteLimitAbs = AZStd::abs(absoluteLimit);
       return AZStd::clamp(value, -absoluteLimitAbs, absoluteLimitAbs);
   }
} // namespace ROS2::VehicleDynamics
