/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FilePath.h"

namespace ROS2
{
    namespace Utils
    {
        AZStd::string GetCapitalizedExtension(const AZ::IO::Path& filename)
        {
            AZStd::string extension{ filename.Extension().Native() };
            AZStd::to_upper(extension.begin(), extension.end());
            return extension;
        }

        bool IsFileXacro(const AZ::IO::Path& filename)
        {
            return filename.HasExtension() && GetCapitalizedExtension(filename) == ".XACRO";
        }

        bool IsFileUrdf(const AZ::IO::Path& filename)
        {
            return filename.HasExtension() && GetCapitalizedExtension(filename) == ".URDF";
        }
    } // namespace Utils
} // namespace ROS2